"""DOCX 後端。

作法是「原地換字」：走訪每一個段落，把譯文寫回原本那個 run，
段落樣式、表格、頁首頁尾、清單編號、圖片位置全部保留。
內嵌圖片則整張換掉（尺寸不變，所以版面不會跑掉）。
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Iterator, List, Optional, Tuple

import docx
from docx.document import Document as DocxDocument
from docx.parts.image import ImagePart
from docx.table import Table
from docx.text.hyperlink import Hyperlink
from docx.text.paragraph import Paragraph
from docx.text.run import Run

from .config import Config
from .imagetrans import FontBook, redraw, reencode_as
from .llm import Segment, Translator
from .report import FileStats


def _iter_paragraphs(container) -> Iterator[Paragraph]:
    """段落、表格（含巢狀表格）內的段落，依文件順序走訪。"""
    for paragraph in getattr(container, "paragraphs", []):
        yield paragraph
    for table in getattr(container, "tables", []):
        for row in table.rows:
            for cell in row.cells:
                yield from _iter_paragraphs(cell)


def _all_paragraphs(doc: DocxDocument) -> Iterator[Paragraph]:
    yield from _iter_paragraphs(doc)
    for section in doc.sections:
        for name in ("header", "footer", "first_page_header", "first_page_footer",
                     "even_page_header", "even_page_footer"):
            part = getattr(section, name, None)
            if part is not None:
                yield from _iter_paragraphs(part)


def _chunks(paragraph: Paragraph) -> List[Tuple[str, List[Run]]]:
    """把段落切成可替換的單位。

    連續的一般 run 併成一塊（這樣模型看到的是完整句子），超連結各自一塊
    （才不會把連結的顯示文字併掉、造成重複或連結失效）。
    """
    result: List[Tuple[str, List[Run]]] = []
    pending: List[Run] = []
    for item in paragraph.iter_inner_content():
        if isinstance(item, Run):
            pending.append(item)
        elif isinstance(item, Hyperlink):
            if pending:
                result.append(("runs", pending))
                pending = []
            if item.runs:
                result.append(("link", list(item.runs)))
    if pending:
        result.append(("runs", pending))
    return [(kind, runs) for kind, runs in result if runs]


def _translatable(text: str) -> bool:
    return bool(text.strip()) and any(ch.isalpha() for ch in text)


def _write_back(runs: List[Run], text: str) -> None:
    """把譯文寫進第一個 run（保留它的字型樣式），其餘清空。"""
    runs[0].text = text
    for run in runs[1:]:
        run.text = ""


def translate_docx(path: str, out_path: str, translator: Translator, cfg: Config,
                   book: FontBook, log: Callable[[str], None]) -> FileStats:
    stats = FileStats(path=path, output=out_path)
    doc = docx.Document(path)

    # --- 1. 收集文字 ---
    segments: List[Segment] = []
    targets = {}
    for p_idx, paragraph in enumerate(_all_paragraphs(doc)):
        chunk_list = _chunks(paragraph)
        para_text = paragraph.text.strip()
        for c_idx, (_kind, runs) in enumerate(chunk_list):
            text = "".join(r.text for r in runs)
            if not _translatable(text):
                continue
            seg_id = f"p{p_idx}c{c_idx}"
            context = para_text if len(chunk_list) > 1 and para_text != text else ""
            segments.append(Segment(id=seg_id, text=text, context=context))
            targets[seg_id] = runs
    stats.segments = len(segments)

    if cfg.dry_run:
        stats.est_input_tokens = translator.estimate_tokens(segments)
        stats.notes.append(f"dry-run：{len(segments)} 段文字待譯")
    elif segments:
        translations = translator.translate_segments(segments)
        for seg in segments:
            new_text = translations.get(seg.id)
            if new_text and new_text != seg.text:
                _write_back(targets[seg.id], new_text)
                stats.translated += 1

    # --- 2. 圖片 ---
    image_parts = [p for p in doc.part.package.iter_parts() if isinstance(p, ImagePart)]
    stats.images_seen = len(image_parts)
    if cfg.translate_images and cfg.image_mode == "redraw" and not cfg.dry_run:
        stats.images_translated = _translate_images(
            image_parts, translator, cfg, book, log
        )
    elif image_parts:
        stats.notes.append("圖片未處理（--image-mode skip 或 --dry-run）")

    if not cfg.dry_run:
        doc.save(out_path)
    return stats


def _translate_images(parts: List[ImagePart], translator: Translator, cfg: Config,
                      book: FontBook, log: Callable[[str], None]) -> int:
    def handle(indexed) -> int:
        idx, part = indexed
        blob = part.blob
        try:
            width = part.image.px_width
            height = part.image.px_height
        except Exception:
            width = height = cfg.image_min_side + 1
        if min(width, height) < cfg.image_min_side:
            return 0
        regions = translator.translate_image(
            blob, part.content_type, where=f"文件中的第 {idx + 1} 張圖"
        )
        if not regions:
            return 0
        debug = None
        if cfg.image_debug_dir:
            debug = f"{cfg.image_debug_dir.rstrip('/')}/docx-img{idx + 1}.png"
        new_bytes = redraw(blob, regions, book, debug_path=debug)
        if not new_bytes:
            return 0
        part._blob = reencode_as(new_bytes, part.content_type)
        log(f"  圖片 {idx + 1}/{len(parts)}：重繪 {len(regions)} 段文字")
        return 1

    if not parts:
        return 0
    log(f"  圖片：{len(parts)} 張待處理")
    with ThreadPoolExecutor(max_workers=cfg.concurrency) as pool:
        return sum(pool.map(handle, enumerate(parts)))
