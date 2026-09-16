"""圖片重繪：把譯文畫回原本文字所在的位置。

流程是「量 → 蓋 → 寫」：
  1. 量：從 bbox 邊緣取樣，推出底色與字色；
  2. 蓋：用底色填掉原本的文字；
  3. 寫：自動挑字級與斷行，把譯文寫回同一個框內。

這是純像素層的處理，不是重新排版，所以底色複雜（文字壓在照片上）時
會看得出補丁痕跡 —— 這是這個模式的已知取捨，可用 --image-mode skip 關掉。
"""

from __future__ import annotations

import io
import re
import threading
from collections import Counter
from typing import Dict, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont

from .llm import TextRegion

# 會被逐字斷行的文字（CJK、假名、諺文）
_CJK = r"⺀-〿぀-ヿ㐀-䶿一-鿿가-힯豈-﫿＀-｠"
_UNIT_RE = re.compile(f"[{_CJK}]|[^\\s{_CJK}]+|\\s+")

_MIN_FONT = 6
_LINE_SPACING = 1.12


class FontBook:
    """同一個字型檔的不同字級快取。

    PIL 的字型物件不保證可以跨執行緒共用，所以這裡附一把鎖，
    由 redraw() 在整段繪圖期間持有（繪圖是 CPU 密集工作，序列化不會變慢）。
    """

    def __init__(self, font_path: Optional[str]):
        self.path = font_path
        self.lock = threading.Lock()
        self._cache: Dict[int, ImageFont.FreeTypeFont] = {}

    def get(self, size: int) -> ImageFont.ImageFont:
        size = max(1, int(size))
        if size not in self._cache:
            if self.path:
                self._cache[size] = ImageFont.truetype(self.path, size)
            else:
                self._cache[size] = ImageFont.load_default()
        return self._cache[size]


# ----------------------------------------------------------------------
# 顏色
# ----------------------------------------------------------------------
def _estimate_colors(img: Image.Image, box: Tuple[int, int, int, int]):
    """從框內取樣推出 (底色, 字色)。"""
    crop = img.crop(box).convert("RGB")
    w, h = crop.size
    # 用 tobytes() 而不是 getdata()：後者在 Pillow 14 會被移除。
    raw = crop.tobytes()
    pixels = [tuple(raw[i:i + 3]) for i in range(0, len(raw), 3)]
    if not pixels:
        return (255, 255, 255), (0, 0, 0)

    ring_depth = max(1, min(2, w // 4, h // 4))
    ring = [
        pixels[y * w + x]
        for y in range(h)
        for x in range(w)
        if x < ring_depth or y < ring_depth or x >= w - ring_depth or y >= h - ring_depth
    ]
    bg = Counter(ring or pixels).most_common(1)[0][0]

    scored = [(sum((c[i] - bg[i]) ** 2 for i in range(3)), c) for c in pixels]
    max_dist = max(d for d, _ in scored)
    if max_dist < 1500:                      # 框內幾乎沒有對比 → 用底色的反差色
        luma = 0.299 * bg[0] + 0.587 * bg[1] + 0.114 * bg[2]
        return bg, ((0, 0, 0) if luma > 140 else (255, 255, 255))

    far = [c for d, c in scored if d >= max_dist * 0.55]
    fg = tuple(sum(c[i] for c in far) // len(far) for i in range(3))
    return bg, fg


# ----------------------------------------------------------------------
# 排字
# ----------------------------------------------------------------------
def _measure(draw: ImageDraw.ImageDraw, text: str, font) -> float:
    if not text:
        return 0.0
    return draw.textlength(text, font=font)


def _wrap(draw: ImageDraw.ImageDraw, text: str, font, max_width: float) -> List[str]:
    """依可用寬度斷行；CJK 逐字斷，拉丁文字依詞斷。"""
    lines: List[str] = []
    for raw_line in text.split("\n"):
        if not raw_line.strip():
            lines.append("")
            continue
        current = ""
        for unit in _UNIT_RE.findall(raw_line):
            candidate = current + unit
            if current and _measure(draw, candidate.rstrip(), font) > max_width:
                lines.append(current.rstrip())
                current = "" if unit.isspace() else unit
            else:
                current = candidate
        if current.strip() or not lines:
            lines.append(current.rstrip())
    return lines or [""]


def _fit(draw: ImageDraw.ImageDraw, text: str, book: FontBook,
         box_w: float, box_h: float) -> Tuple[object, List[str], float]:
    """二分搜尋出塞得進框內的最大字級，回傳 (font, lines, line_height)。"""
    lo, hi = _MIN_FONT, max(_MIN_FONT, int(box_h))
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        font = book.get(mid)
        lines = _wrap(draw, text, font, box_w)
        ascent, descent = font.getmetrics()
        line_h = (ascent + descent) * _LINE_SPACING
        fits_w = all(_measure(draw, ln, font) <= box_w for ln in lines)
        if fits_w and line_h * len(lines) <= box_h:
            best = (font, lines, line_h)
            lo = mid + 1
        else:
            hi = mid - 1
    if best is None:                          # 框太小，用最小字級硬塞
        font = book.get(_MIN_FONT)
        lines = _wrap(draw, text, font, box_w)
        ascent, descent = font.getmetrics()
        best = (font, lines, (ascent + descent) * _LINE_SPACING)
    return best


def _draw_horizontal(draw, region: TextRegion, box, book: FontBook, fg) -> None:
    x0, y0, x1, y1 = box
    box_w, box_h = x1 - x0, y1 - y0
    font, lines, line_h = _fit(draw, region.translation, book, box_w, box_h)
    total_h = line_h * len(lines)
    y = y0 + max(0.0, (box_h - total_h) / 2)
    for line in lines:
        width = _measure(draw, line, font)
        if region.align == "center":
            x = x0 + (box_w - width) / 2
        elif region.align == "right":
            x = x1 - width
        else:
            x = x0
        draw.text((x, y), line, font=font, fill=fg)
        y += line_h


def _draw_vertical(draw, region: TextRegion, box, book: FontBook, fg) -> None:
    """直排：由右至左、由上而下逐字寫。"""
    x0, y0, x1, y1 = box
    box_w, box_h = x1 - x0, y1 - y0
    chars = [c for c in region.translation if c != "\n"]
    if not chars:
        return
    columns = 1
    size = _MIN_FONT
    while True:
        per_col = max(1, int(box_h // max(1, size)))
        columns = max(1, -(-len(chars) // per_col))
        if columns * size > box_w or size >= box_h:
            size = max(_MIN_FONT, size - 1)
            break
        size += 1
    font = book.get(size)
    per_col = max(1, int(box_h // size))
    columns = max(1, -(-len(chars) // per_col))
    for col in range(columns):
        x = x1 - (col + 1) * size
        chunk = chars[col * per_col:(col + 1) * per_col]
        for row, ch in enumerate(chunk):
            draw.text((x, y0 + row * size), ch, font=font, fill=fg)


# ----------------------------------------------------------------------
# 對外
# ----------------------------------------------------------------------
def redraw(image_bytes: bytes, regions: Sequence[TextRegion], book: FontBook,
           debug_path: Optional[str] = None) -> Optional[bytes]:
    """把 regions 的譯文畫回圖上，回傳新的圖片位元組；沒有可畫的就回 None。"""
    if not regions:
        return None
    with book.lock:
        return _redraw_locked(image_bytes, regions, book, debug_path)


def _redraw_locked(image_bytes: bytes, regions: Sequence[TextRegion], book: FontBook,
                   debug_path: Optional[str]) -> Optional[bytes]:
    try:
        img = Image.open(io.BytesIO(image_bytes))
        img.load()
    except Exception:
        return None

    fmt = (img.format or "PNG").upper()
    has_alpha = img.mode in ("RGBA", "LA") or (
        img.mode == "P" and "transparency" in img.info
    )
    work = img.convert("RGBA" if has_alpha else "RGB")
    draw = ImageDraw.Draw(work)
    w, h = work.size

    drawn = 0
    for region in regions:
        bx0, by0, bx1, by1 = region.bbox
        # 模型給的框常常「剛好差一點點」，所以按比例外擴一小圈，
        # 避免原文的邊緣（字母末端、下緣）沒被蓋掉。
        pad_x = max(1, round((bx1 - bx0) * w * 0.015))
        pad_y = max(1, round((by1 - by0) * h * 0.08))
        box = (
            max(0, int(bx0 * w) - pad_x),
            max(0, int(by0 * h) - pad_y),
            min(w, int(round(bx1 * w)) + pad_x),
            min(h, int(round(by1 * h)) + pad_y),
        )
        if box[2] - box[0] < 4 or box[3] - box[1] < 6:
            continue
        bg, fg = _estimate_colors(work, box)
        fill = bg + (255,) if has_alpha else bg
        colour = fg + (255,) if has_alpha else fg
        draw.rectangle(box, fill=fill)
        if region.vertical:
            _draw_vertical(draw, region, box, book, colour)
        else:
            _draw_horizontal(draw, region, box, book, colour)
        drawn += 1

    if drawn == 0:
        return None

    if debug_path:
        overlay = img.convert("RGB")
        odraw = ImageDraw.Draw(overlay)
        for region in regions:
            bx0, by0, bx1, by1 = region.bbox
            odraw.rectangle(
                (bx0 * w, by0 * h, bx1 * w, by1 * h), outline=(255, 0, 0), width=2
            )
        overlay.save(debug_path)

    buf = io.BytesIO()
    if fmt in ("JPEG", "JPG"):
        work.convert("RGB").save(buf, format="JPEG", quality=92, subsampling=0)
    elif fmt in ("PNG", "GIF", "WEBP", "BMP", "TIFF"):
        work.save(buf, format="PNG" if fmt in ("GIF", "BMP") else fmt)
    else:
        work.convert("RGB").save(buf, format="PNG")
    return buf.getvalue()


def reencode_as(image_bytes: bytes, content_type: str) -> bytes:
    """把圖片轉成指定的 content-type，讓它能原地替換掉容器裡的舊圖。"""
    subtype = content_type.rsplit("/", 1)[-1].lower()
    target = {"jpeg": "JPEG", "jpg": "JPEG", "png": "PNG", "gif": "GIF",
              "bmp": "BMP", "tiff": "TIFF", "webp": "WEBP"}.get(subtype)
    if target is None:
        return image_bytes
    img = Image.open(io.BytesIO(image_bytes))
    img.load()
    if (img.format or "").upper() == target:
        return image_bytes
    if target in ("JPEG", "BMP"):
        img = img.convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format=target, **({"quality": 92} if target == "JPEG" else {}))
    return buf.getvalue()
