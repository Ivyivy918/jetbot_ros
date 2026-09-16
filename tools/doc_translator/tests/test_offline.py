"""離線測試：不呼叫 API，用假的 Translator 驗證整條管線。

驗的是「文件處理」而不是「翻譯品質」：
  · DOCX / PDF 的文字有沒有被換成譯文、樣式與表格有沒有保住；
  · 內嵌圖片有沒有被重繪後的版本換掉；
  · 圖片重繪本身會不會把框內填成底色再寫上譯文。

執行：  python -m pytest tools/doc_translator/tests -q
或直接：python tools/doc_translator/tests/test_offline.py
"""

from __future__ import annotations

import io
import sys
from pathlib import Path

import docx
import pymupdf
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.doc_translator.config import Config                      # noqa: E402
from tools.doc_translator.docx_backend import translate_docx        # noqa: E402
from tools.doc_translator.fonts import find_font                    # noqa: E402
from tools.doc_translator.imagetrans import FontBook, redraw        # noqa: E402
from tools.doc_translator.llm import Segment, TextRegion, Translator  # noqa: E402
from tools.doc_translator.pdf_backend import translate_pdf          # noqa: E402

MARK = "【譯】"


class FakeTranslator(Translator):
    """把每段文字前面加上標記，並宣稱每張圖的左上角有一段文字。"""

    def translate_segments(self, segments):
        return {s.id: MARK + s.text.replace("\n", "\n") for s in segments}

    def translate_image(self, image_bytes, media_type, where=""):
        return [TextRegion(text="Speed Limit", translation="速限標示",
                           bbox=(0.05, 0.10, 0.95, 0.45), align="center")]

    def estimate_tokens(self, segments):
        return sum(len(s.text) for s in segments) // 3


def make_image(path: Path) -> None:
    img = Image.new("RGB", (480, 200), (250, 248, 240))
    draw = ImageDraw.Draw(img)
    draw.rectangle((10, 10, 470, 190), outline=(20, 60, 140), width=3)
    draw.text((40, 40), "Speed Limit", fill=(20, 20, 20))
    img.save(path)


def make_docx(path: Path, image_path: Path) -> None:
    doc = docx.Document()
    doc.add_heading("Robot Operating Manual", level=1)
    doc.add_paragraph("Connect the battery before powering the motor driver.")
    para = doc.add_paragraph("This text is ")
    run = para.add_run("partially bold")
    run.bold = True
    para.add_run(" in the middle.")
    table = doc.add_table(rows=2, cols=2)
    table.cell(0, 0).text = "Parameter"
    table.cell(0, 1).text = "Value"
    table.cell(1, 0).text = "Wheel radius"
    table.cell(1, 1).text = "32 mm"
    doc.add_picture(str(image_path))
    doc.sections[0].header.paragraphs[0].text = "Confidential draft"
    doc.save(path)


def make_pdf(path: Path, image_path: Path) -> None:
    doc = pymupdf.open()
    page = doc.new_page()
    page.insert_textbox(pymupdf.Rect(60, 60, 520, 110),
                        "Robot Operating Manual", fontsize=20, fontname="helv")
    page.insert_textbox(pymupdf.Rect(60, 120, 520, 200),
                        "Connect the battery before powering the motor driver. "
                        "Check that the wheels spin freely.",
                        fontsize=11, fontname="helv")
    page.insert_image(pymupdf.Rect(60, 220, 420, 370), filename=str(image_path))
    doc.save(path)
    doc.close()


def _config(**kw) -> Config:
    return Config(target_language="繁體中文", use_cache=False, concurrency=2, **kw)


# ----------------------------------------------------------------------
def test_redraw_paints_over_original(tmp_path: Path) -> None:
    image_path = tmp_path / "sign.png"
    make_image(image_path)
    original = image_path.read_bytes()
    book = FontBook(find_font("繁體中文"))
    region = TextRegion(text="Speed Limit", translation="速限標示",
                        bbox=(0.05, 0.15, 0.95, 0.40), align="center")

    out = redraw(original, [region], book)
    assert out and out != original

    before = Image.open(io.BytesIO(original)).convert("RGB")
    after = Image.open(io.BytesIO(out)).convert("RGB")
    assert before.size == after.size
    box = (int(0.05 * 480), int(0.15 * 200), int(0.95 * 480), int(0.40 * 200))
    assert before.crop(box).tobytes() != after.crop(box).tobytes()
    # 框外不該被動到
    assert before.getpixel((5, 195)) == after.getpixel((5, 195))


def test_docx_roundtrip(tmp_path: Path) -> None:
    image_path = tmp_path / "sign.png"
    src = tmp_path / "manual.docx"
    dst = tmp_path / "manual.zh.docx"
    make_image(image_path)
    make_docx(src, image_path)

    cfg = _config()
    translator = FakeTranslator(cfg)
    stats = translate_docx(str(src), str(dst), translator, cfg,
                           FontBook(find_font("繁體中文")), lambda m: None)

    assert dst.exists()
    assert stats.translated > 0 and stats.translated == stats.segments
    assert stats.images_seen == 1 and stats.images_translated == 1

    out = docx.Document(dst)
    texts = [p.text for p in out.paragraphs]
    assert any(t.startswith(MARK + "Robot Operating Manual") for t in texts)
    # 表格與頁首也要翻到
    assert out.tables[0].cell(1, 0).text.startswith(MARK)
    assert out.sections[0].header.paragraphs[0].text.startswith(MARK)
    # 粗體段落：三個 run 併成一段譯文，但第一個 run 的樣式保留
    mixed = [p for p in out.paragraphs if "in the middle" in p.text]
    assert mixed and mixed[0].text == MARK + "This text is partially bold in the middle."
    # 圖片換掉了，而且尺寸不變（版面才不會跑掉）
    from docx.parts.image import ImagePart
    part = next(p for p in out.part.package.iter_parts() if isinstance(p, ImagePart))
    assert part.blob != image_path.read_bytes()
    assert Image.open(io.BytesIO(part.blob)).size == (480, 200)


def test_pdf_roundtrip(tmp_path: Path) -> None:
    image_path = tmp_path / "sign.png"
    src = tmp_path / "manual.pdf"
    dst = tmp_path / "manual.zh.pdf"
    make_image(image_path)
    make_pdf(src, image_path)

    cfg = _config()
    translator = FakeTranslator(cfg)
    stats = translate_pdf(str(src), str(dst), translator, cfg,
                          FontBook(find_font("繁體中文")), lambda m: None)

    assert dst.exists()
    assert stats.segments >= 2
    assert stats.translated == stats.segments
    assert stats.images_seen == 1 and stats.images_translated == 1

    out = pymupdf.open(dst)
    text = out[0].get_text()
    assert MARK in text
    assert "Robot Operating Manual" not in text.replace(MARK, "")
    assert len(out[0].get_images(full=True)) == 1
    out.close()


def test_dry_run_does_not_write(tmp_path: Path) -> None:
    image_path = tmp_path / "sign.png"
    src = tmp_path / "manual.docx"
    dst = tmp_path / "manual.zh.docx"
    make_image(image_path)
    make_docx(src, image_path)

    cfg = _config(dry_run=True)
    stats = translate_docx(str(src), str(dst), FakeTranslator(cfg), cfg,
                           FontBook(None), lambda m: None)
    assert not dst.exists()
    assert stats.segments > 0 and stats.translated == 0
    assert stats.est_input_tokens > 0


def test_batching_respects_limits() -> None:
    cfg = _config(max_chars_per_batch=50, max_segments_per_batch=3)
    translator = FakeTranslator(cfg)
    segments = [Segment(id=f"s{i}", text="x" * 20) for i in range(7)]
    batches = translator._batch(segments)
    assert all(len(b) <= 3 for b in batches)
    assert sum(len(b) for b in batches) == 7


if __name__ == "__main__":
    import tempfile
    import traceback

    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            if fn.__code__.co_argcount:
                with tempfile.TemporaryDirectory() as tmp:
                    fn(Path(tmp))
            else:
                fn()
            print(f"  ok   {name}")
        except Exception:
            failures += 1
            print(f"  FAIL {name}")
            traceback.print_exc()
    print("全部通過" if not failures else f"{failures} 項失敗")
    raise SystemExit(1 if failures else 0)
