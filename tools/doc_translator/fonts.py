"""字型挑選。

圖片重繪與 PDF 內文都需要一個「能畫出目標語言」的字型；
預設值猜錯時，使用者可以用 --font / --pdf-font 指定。
"""

from __future__ import annotations

import os
import re
import subprocess
from typing import List, Optional

# 依序嘗試：先找涵蓋 CJK 的字型，再退回拉丁字型。
_CANDIDATES: List[str] = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf",
    "/System/Library/Fonts/PingFang.ttc",
    "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "C:/Windows/Fonts/msjh.ttc",
    "C:/Windows/Fonts/msyh.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/Library/Fonts/Arial Unicode.ttf",
    "C:/Windows/Fonts/arial.ttf",
]

# PyMuPDF 內建的 CJK base-14 延伸字型，不需要外部檔案即可嵌入 PDF。
_PDF_BUILTIN = {
    "zh-hant": "china-t",
    "zh-hans": "china-s",
    "ja": "japan",
    "ko": "korea",
    "latin": "helv",
}


def classify_language(target_language: str) -> str:
    """把使用者寫的目標語言字串歸類成字型類別。"""
    t = target_language.lower()
    if re.search(r"繁體|正體|zh-?tw|zh-?hant|traditional", t):
        return "zh-hant"
    if re.search(r"简体|簡體|zh-?cn|zh-?hans|simplified", t):
        return "zh-hans"
    if re.search(r"中文|chinese|zh", t):
        return "zh-hant"
    if re.search(r"日本語|日文|japanese|ja\b|jp", t):
        return "ja"
    if re.search(r"한국|韓文|韓語|korean|ko\b|kr", t):
        return "ko"
    return "latin"


def _fc_match(lang_class: str) -> Optional[str]:
    """用 fontconfig 問系統要一個支援該語言的字型（有 fc-match 時才用得上）。"""
    lang = {"zh-hant": "zh-tw", "zh-hans": "zh-cn", "ja": "ja", "ko": "ko"}.get(lang_class)
    if not lang:
        return None
    try:
        out = subprocess.run(
            ["fc-match", "-f", "%{file}", f":lang={lang}"],
            capture_output=True, text=True, timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    path = out.stdout.strip()
    return path if path and os.path.exists(path) else None


def find_font(target_language: str, override: Optional[str] = None) -> Optional[str]:
    """找一個可以用 Pillow 畫出目標語言的字型檔路徑。"""
    if override:
        if not os.path.exists(override):
            raise FileNotFoundError(f"找不到指定的字型檔：{override}")
        return override
    lang_class = classify_language(target_language)
    found = _fc_match(lang_class)
    if found:
        return found
    for path in _CANDIDATES:
        if os.path.exists(path):
            return path
    return None


def pdf_font(target_language: str, override_file: Optional[str] = None) -> tuple:
    """回傳 (fontname, fontfile)，可直接餵給 PyMuPDF 的 insert_textbox。

    有指定字型檔就用它（嵌入子集）；否則用 PyMuPDF 內建的 CJK 字型，
    不必依賴系統上裝了什麼字型。
    """
    if override_file:
        if not os.path.exists(override_file):
            raise FileNotFoundError(f"找不到指定的 PDF 字型檔：{override_file}")
        return "DocTr", override_file
    return _PDF_BUILTIN[classify_language(target_language)], None
