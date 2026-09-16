"""單一檔案的處理統計，以及結束時的花費摘要。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .config import Config
from .llm import Usage


@dataclass
class FileStats:
    path: str
    output: str = ""
    segments: int = 0
    translated: int = 0
    images_seen: int = 0
    images_translated: int = 0
    est_input_tokens: int = 0
    notes: List[str] = field(default_factory=list)
    error: str = ""

    def line(self) -> str:
        if self.error:
            return f"  ✗ {self.path}：{self.error}"
        if self.est_input_tokens and not self.translated:
            return (f"  · {self.path}：文字片段 {self.segments} 段、"
                    f"圖片 {self.images_seen} 張")
        return (f"  ✓ {self.path} → {self.output}\n"
                f"      文字片段 {self.translated}/{self.segments}、"
                f"圖片 {self.images_translated}/{self.images_seen} 張重繪")


def summarize(cfg: Config, usage: Usage, stats: List[FileStats],
              cache_hits: int) -> str:
    price_in, price_out = cfg.price()
    lines = ["", "=" * 56, "試算結果" if cfg.dry_run else "翻譯完成"]
    for s in stats:
        lines.append(s.line())
        for note in s.notes:
            lines.append(f"      · {note}")
    lines.append("-" * 56)
    estimated = sum(s.est_input_tokens for s in stats)
    if estimated:
        price_in, _ = cfg.price()
        lines.append(f"預估輸入 tokens：{estimated:,}"
                     + (f"（約 US${estimated * price_in / 1_000_000:.4f}，尚未計輸出）"
                        if price_in else ""))
    if usage.requests:
        lines.append(
            f"API 呼叫 {usage.requests} 次｜輸入 {usage.input_tokens:,} tokens"
            f"（快取命中 {usage.cache_read_tokens:,}）｜輸出 {usage.output_tokens:,} tokens"
        )
        if price_in:
            lines.append(f"估計花費：約 US${usage.cost(price_in, price_out):.4f}"
                         f"（依 {cfg.model} 的公開價格，未計入快取折扣）")
    if cache_hits:
        lines.append(f"本機快取命中 {cache_hits} 次（這些片段沒有再次呼叫 API）")
    lines.append("=" * 56)
    return "\n".join(lines)
