"""翻譯工作的設定物件。

設定來源的優先順序（後者覆蓋前者）：
    預設值  <  YAML 設定檔（--config）  <  命令列參數  <  一開始的互動輸入
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, Optional

DEFAULT_MODEL = "claude-opus-5"

# USD / 1M tokens，(input, output)。用於 --dry-run 與結束時的花費估算。
PRICING: Dict[str, tuple] = {
    "claude-fable-5-1": (10.0, 50.0),
    "claude-opus-5": (5.0, 25.0),
    "claude-opus-4-8": (5.0, 25.0),
    "claude-sonnet-5": (2.0, 10.0),
    "claude-haiku-4-5": (1.0, 5.0),
}

SUPPORTED_SUFFIXES = {".pdf", ".docx"}


@dataclass
class Config:
    """一次翻譯工作的全部設定。"""

    # --- 翻譯內容 ---
    target_language: str = "繁體中文"
    source_language: str = "auto"
    instructions: str = ""              # 使用者一開始提出的特殊要求
    glossary: Dict[str, str] = field(default_factory=dict)

    # --- 模型 ---
    model: str = DEFAULT_MODEL
    effort: str = "low"                 # 文字批次：翻譯是密集但單純的工作
    image_effort: str = "medium"        # 圖片：要讀圖 + 估座標，值得多想一點
    max_output_tokens: int = 16000

    # --- 圖片 ---
    translate_images: bool = True
    image_mode: str = "redraw"          # redraw = 譯文蓋回原位；skip = 保留原圖
    image_min_side: int = 48            # 小於此邊長的圖（項目符號、圖示）直接略過
    image_debug_dir: Optional[str] = None

    # --- 批次與並行 ---
    max_chars_per_batch: int = 3000
    max_segments_per_batch: int = 40
    concurrency: int = 4

    # --- 字型 ---
    font_path: Optional[str] = None     # 圖片重繪用的 TTF/OTF
    pdf_font_file: Optional[str] = None # PDF 內文用的 TTF/OTF（留空則用內建 CJK 字型）

    # --- 其他 ---
    cache_path: Optional[str] = None
    use_cache: bool = True
    dry_run: bool = False
    verbose: bool = False

    # ------------------------------------------------------------------
    def price(self) -> tuple:
        return PRICING.get(self.model, (0.0, 0.0))

    def merged(self, **overrides: Any) -> "Config":
        """回傳套用 overrides（忽略 None）之後的新 Config。"""
        clean = {k: v for k, v in overrides.items() if v is not None}
        return replace(self, **clean)


def _load_mapping(path: Path) -> Dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in (".yaml", ".yml"):
        import yaml  # PyYAML 為選用相依；只有讀 YAML 時才需要
        data = yaml.safe_load(text) or {}
    else:
        data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError(f"{path} 的內容必須是一個物件／字典")
    return data


def load_config_file(path: str) -> Dict[str, Any]:
    """讀取 --config 指定的 YAML/JSON，只保留 Config 認得的欄位。"""
    data = _load_mapping(Path(path))
    known = {f.name for f in Config.__dataclass_fields__.values()}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"設定檔含有未知欄位：{', '.join(sorted(unknown))}")
    return data


def load_glossary(path: str) -> Dict[str, str]:
    """讀取術語表。支援 YAML/JSON 的 {原文: 譯文}，以及每行 `原文,譯文` 的 CSV。"""
    p = Path(path)
    if p.suffix.lower() == ".csv":
        import csv
        out: Dict[str, str] = {}
        with p.open(encoding="utf-8", newline="") as fh:
            for row in csv.reader(fh):
                if len(row) >= 2 and row[0].strip():
                    out[row[0].strip()] = row[1].strip()
        return out
    return {str(k): str(v) for k, v in _load_mapping(p).items()}


def default_cache_path() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(base, "doc_translator", "translations.sqlite3")
