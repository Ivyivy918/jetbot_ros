"""PDF 後端。

和圖片用同一套「蓋回原位」的想法，只是在向量層做：
逐個文字區塊取出原文與其外框，翻譯後把原本的文字塗銷（redaction），
再把譯文以自動縮放的字級寫回同一個框裡。表格線、向量圖與圖片都不動。

限制（設計上的取捨，README 有寫）：
  * 區塊內的混合樣式（單字粗體、上標）會統一成該區塊的主要樣式；
  * 譯文比原文長時會自動縮小字級，縮到下限仍放不下就會截斷；
  * 掃描版 PDF 沒有文字層，會整頁走圖片重繪的路徑。
"""

from __future__ import annotations

from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Dict, List, Optional, Tuple

import pymupdf

from .config import Config
from .imagetrans import FontBook, redraw
from .llm import Segment, Translator
from .report import FileStats

_MIN_PDF_FONT = 4.0


def _translatable(text: str) -> bool:
    return bool(text.strip()) and any(ch.isalpha() for ch in text)


class _Block:
    __slots__ = ("id", "page", "rect", "text", "size", "color", "align")

    def __init__(self, id_: str, page: int, rect, text: str, size: float,
                 color, align: int):
        self.id = id_
        self.page = page
        self.rect = rect
        self.text = text
        self.size = size
        self.color = color
        self.align = align


def _collect_blocks(page, page_index: int) -> List[_Block]:
    blocks: List[_Block] = []
    data = page.get_text("dict")
    page_rect = page.rect
    for b_idx, block in enumerate(data.get("blocks", [])):
        if block.get("type") != 0:
            continue
        lines = block.get("lines", [])
        if not lines:
            continue
        # 只處理水平排列的文字；直排／旋轉文字保留原樣，避免畫壞。
        if any(tuple(round(v, 2) for v in ln.get("dir", (1, 0))) != (1.0, 0.0)
               for ln in lines):
            continue
        line_texts, sizes, colors = [], [], []
        for line in lines:
            spans = line.get("spans", [])
            line_texts.append("".join(s.get("text", "") for s in spans))
            for span in spans:
                if span.get("text", "").strip():
                    sizes.append(span.get("size", 11.0))
                    colors.append(span.get("color", 0))
        text = "\n".join(line_texts).strip("\n")
        if not _translatable(text) or not sizes:
            continue
        rect = pymupdf.Rect(block["bbox"])
        centre = (rect.x0 + rect.x1) / 2
        page_centre = (page_rect.x0 + page_rect.x1) / 2
        centred = abs(centre - page_centre) < page_rect.width * 0.04 and \
            rect.width < page_rect.width * 0.8
        blocks.append(_Block(
            id_=f"p{page_index}b{b_idx}",
            page=page_index,
            rect=rect,
            text=text,
            size=max(sizes),
            color=pymupdf.sRGB_to_pdf(Counter(colors).most_common(1)[0][0]),
            align=pymupdf.TEXT_ALIGN_CENTER if centred else pymupdf.TEXT_ALIGN_LEFT,
        ))
    return blocks


def _insert(page, block: _Block, text: str, fontname: str,
            fontfile: Optional[str]) -> bool:
    """把譯文寫進原本的框；放不下就逐步縮小字級。回傳是否成功。"""
    rect = pymupdf.Rect(block.rect)
    rect.x1 = min(rect.x1 + 2.0, page.rect.x1)          # 給換行一點餘裕
    size = block.size
    while size >= _MIN_PDF_FONT:
        try:
            spare = page.insert_textbox(
                rect, text, fontname=fontname, fontfile=fontfile,
                fontsize=size, color=block.color, align=block.align,
            )
        except Exception:
            return False
        if spare >= 0:
            return True
        size -= max(0.5, size * 0.08)
    # 已到最小字級仍放不下：用最小字級硬寫（PyMuPDF 會截斷超出的部分）
    try:
        page.insert_textbox(
            rect, text, fontname=fontname, fontfile=fontfile,
            fontsize=_MIN_PDF_FONT, color=block.color, align=block.align,
        )
    except Exception:
        return False
    return True


def translate_pdf(path: str, out_path: str, translator: Translator, cfg: Config,
                  book: FontBook, log: Callable[[str], None]) -> FileStats:
    from .fonts import pdf_font

    stats = FileStats(path=path, output=out_path)
    doc = pymupdf.open(path)
    rotations: Dict[int, int] = {}

    # 先把頁面轉正，讓「讀座標」與「寫座標」在同一個座標系。
    for index, page in enumerate(doc):
        if page.rotation:
            rotations[index] = page.rotation
            page.set_rotation(0)

    # --- 1. 圖片（含掃描頁的整頁影像）---
    if cfg.translate_images and cfg.image_mode == "redraw" and not cfg.dry_run:
        stats.images_seen, stats.images_translated = _translate_images(
            doc, translator, cfg, book, log
        )
    else:
        stats.images_seen = len({x[0] for p in doc for x in p.get_images(full=True)})
        if stats.images_seen:
            stats.notes.append("圖片未處理（--image-mode skip 或 --dry-run）")

    # --- 2. 文字 ---
    blocks: List[_Block] = []
    for index, page in enumerate(doc):
        blocks.extend(_collect_blocks(page, index))
    stats.segments = len(blocks)

    if cfg.dry_run:
        stats.est_input_tokens = translator.estimate_tokens(
            [Segment(id=b.id, text=b.text) for b in blocks])
        stats.notes.append(f"dry-run：{len(blocks)} 個文字區塊待譯")
        doc.close()
        return stats

    if not blocks:
        stats.notes.append("找不到文字層（可能是掃描版 PDF），只處理了圖片")

    translations: Dict[str, str] = {}
    if blocks:
        translations = translator.translate_segments(
            [Segment(id=b.id, text=b.text) for b in blocks]
        )

    fontname, fontfile = pdf_font(cfg.target_language, cfg.pdf_font_file)
    changed = [b for b in blocks
               if translations.get(b.id) and translations[b.id] != b.text]

    by_page: Dict[int, List[_Block]] = {}
    for block in changed:
        by_page.setdefault(block.page, []).append(block)

    for page_index, page_blocks in by_page.items():
        page = doc[page_index]
        for block in page_blocks:
            page.add_redact_annot(block.rect)
        # 只清掉文字：圖片與向量線條（表格框線）原封不動。
        page.apply_redactions(
            images=pymupdf.PDF_REDACT_IMAGE_NONE,
            graphics=pymupdf.PDF_REDACT_LINE_ART_NONE,
            text=pymupdf.PDF_REDACT_TEXT_REMOVE,
        )
        for block in page_blocks:
            if _insert(page, block, translations[block.id], fontname, fontfile):
                stats.translated += 1
            else:
                stats.notes.append(f"第 {page_index + 1} 頁有一個區塊寫不回去")

    for index, rotation in rotations.items():
        doc[index].set_rotation(rotation)

    doc.save(out_path, garbage=3, deflate=True)
    doc.close()
    return stats


def _translate_images(doc, translator: Translator, cfg: Config, book: FontBook,
                      log: Callable[[str], None]) -> Tuple[int, int]:
    """走訪整份 PDF 的內嵌圖片，同一個 xref 只處理一次。"""
    seen: Dict[int, int] = {}          # xref -> 第一次出現的頁碼
    for index, page in enumerate(doc):
        for info in page.get_images(full=True):
            seen.setdefault(info[0], index)
    if not seen:
        return 0, 0

    log(f"  圖片：{len(seen)} 張待處理")

    def handle(item):
        xref, page_index = item
        try:
            info = doc.extract_image(xref)
        except Exception:
            return None
        blob = info.get("image")
        if not blob:
            return None
        if min(info.get("width", 0), info.get("height", 0)) < cfg.image_min_side:
            return None
        media = f"image/{info.get('ext', 'png')}"
        regions = translator.translate_image(
            blob, media, where=f"第 {page_index + 1} 頁的圖"
        )
        if not regions:
            return None
        debug = None
        if cfg.image_debug_dir:
            debug = f"{cfg.image_debug_dir.rstrip('/')}/pdf-x{xref}.png"
        new_bytes = redraw(blob, regions, book, debug_path=debug)
        if not new_bytes:
            return None
        return xref, page_index, new_bytes, len(regions)

    with ThreadPoolExecutor(max_workers=cfg.concurrency) as pool:
        results = [r for r in pool.map(handle, list(seen.items())) if r]

    # 寫回必須在主執行緒序列化進行：PyMuPDF 的文件物件不是執行緒安全的。
    done = 0
    for xref, page_index, new_bytes, region_count in results:
        try:
            doc[page_index].replace_image(xref, stream=new_bytes)
        except Exception as exc:
            log(f"  [警告] 圖片 xref {xref} 無法替換：{exc}")
            continue
        done += 1
        log(f"  第 {page_index + 1} 頁的圖：重繪 {region_count} 段文字")
    return len(seen), done
