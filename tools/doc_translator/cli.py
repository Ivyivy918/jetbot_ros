"""命令列介面。

一開始（互動模式下）會先問目標語言與「翻譯的特殊要求」；
沒有特殊要求就直接留白，交給 Claude 自行判斷。
要排程或寫進腳本時，改用 --instructions / --config 就能完全非互動執行。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import List, Optional

from . import __version__
from .cache import TranslationCache
from .config import (Config, DEFAULT_MODEL, SUPPORTED_SUFFIXES, default_cache_path,
                     load_config_file, load_glossary)
from .fonts import classify_language, find_font
from .imagetrans import FontBook
from .llm import Translator
from .report import FileStats, summarize

_LANG_TAG = {"zh-hant": "zh-TW", "zh-hans": "zh-CN", "ja": "ja", "ko": "ko",
             "latin": "translated"}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="doc_translator",
        description="用 Claude API 翻譯 PDF / DOCX，圖片內的文字也會翻譯並畫回原位。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
範例：
  # 互動模式：先問目標語言與特殊要求，再開始翻
  python -m tools.doc_translator 手冊.pdf

  # 非互動：把特殊要求寫在參數裡
  python -m tools.doc_translator 手冊.pdf -t 英文 \\
      -i "保持技術手冊的口吻；JetBot、ROS2 等產品名不要翻譯"

  # 先估算會花多少錢，不真的呼叫翻譯
  python -m tools.doc_translator *.docx --dry-run
""",
    )
    p.add_argument("inputs", nargs="+", help="要翻譯的 .pdf / .docx 檔")
    p.add_argument("-o", "--output", help="輸出檔名（單一輸入）或輸出目錄")
    p.add_argument("-t", "--to", dest="target_language", help="目標語言（預設 繁體中文）")
    p.add_argument("-f", "--from", dest="source_language", help="來源語言（預設自動偵測）")
    p.add_argument("-i", "--instructions",
                   help="翻譯的特殊要求；用 '-' 表示從 stdin 讀取")
    p.add_argument("--instructions-file", help="從檔案讀取特殊要求")
    p.add_argument("--glossary", help="術語表：YAML / JSON（{原文: 譯文}）或 CSV")
    p.add_argument("--config", help="YAML / JSON 設定檔")

    p.add_argument("--model", help=f"模型（預設 {DEFAULT_MODEL}）")
    p.add_argument("--effort", choices=["low", "medium", "high", "xhigh", "max"],
                   help="文字翻譯的思考強度（預設 low）")
    p.add_argument("--image-effort", choices=["low", "medium", "high", "xhigh", "max"],
                   help="圖片辨識的思考強度（預設 medium）")

    p.add_argument("--image-mode", choices=["redraw", "skip"],
                   help="redraw＝譯文蓋回原位（預設）；skip＝保留原圖")
    p.add_argument("--no-images", action="store_true", help="等同 --image-mode skip")
    p.add_argument("--image-min-side", type=int,
                   help="短邊小於這個像素的圖直接略過（預設 48）")
    p.add_argument("--image-debug-dir",
                   help="把模型框出的文字位置畫成紅框存到這個目錄，方便檢查")

    p.add_argument("--font", help="圖片重繪用的字型檔（TTF/OTF）")
    p.add_argument("--pdf-font", dest="pdf_font_file",
                   help="PDF 內文用的字型檔；不指定則用 PyMuPDF 內建 CJK 字型")

    p.add_argument("--concurrency", type=int, help="同時進行的 API 請求數（預設 4）")
    p.add_argument("--max-chars-per-batch", type=int,
                   help="每批送出的字元上限（預設 3000）")
    p.add_argument("--cache-path", help="譯文快取的 sqlite 路徑")
    p.add_argument("--no-cache", action="store_true", help="停用譯文快取")
    p.add_argument("--dry-run", action="store_true",
                   help="只統計片段數與預估輸入 token，不實際翻譯")
    p.add_argument("-y", "--yes", action="store_true", help="非互動：不詢問、直接開始")
    p.add_argument("-v", "--verbose", action="store_true", help="顯示詳細進度")
    p.add_argument("--version", action="version", version=f"doc_translator {__version__}")
    return p


# ----------------------------------------------------------------------
def _read_instructions(args) -> Optional[str]:
    if args.instructions_file:
        return Path(args.instructions_file).read_text(encoding="utf-8").strip()
    if args.instructions == "-":
        return sys.stdin.read().strip()
    return args.instructions


def _prompt_multiline(prompt: str) -> str:
    """讀多行輸入，空白行結束。"""
    print(prompt)
    lines: List[str] = []
    while True:
        try:
            line = input("  ")
        except EOFError:
            break
        if not line.strip():
            break
        lines.append(line)
    return "\n".join(lines).strip()


def _interactive(cfg: Config, inputs: List[Path]) -> Config:
    print("=" * 56)
    print("  文件翻譯（Claude API）")
    print("=" * 56)
    print("待翻譯檔案：")
    for path in inputs:
        print(f"  · {path}")
    print()

    try:
        answer = input(f"目標語言 [{cfg.target_language}]：").strip()
    except EOFError:
        answer = ""
    target = answer or cfg.target_language

    instructions = cfg.instructions
    if not instructions:
        instructions = _prompt_multiline(
            "\n翻譯的特殊要求（語氣、術語、讀者對象、格式慣例…）\n"
            "直接按 Enter 可略過，略過時由 Claude 自行判斷最合適的譯法。\n"
            "輸入完成後，空一行結束："
        )
    print()
    return cfg.merged(target_language=target, instructions=instructions)


def _resolve_output(path: Path, cfg: Config, output: Optional[str],
                    multiple: bool) -> Path:
    if output:
        out = Path(output)
        if multiple or out.is_dir() or output.endswith(os.sep):
            out.mkdir(parents=True, exist_ok=True)
            return out / f"{path.stem}.{_tag(cfg)}{path.suffix}"
        out.parent.mkdir(parents=True, exist_ok=True)
        return out
    return path.with_name(f"{path.stem}.{_tag(cfg)}{path.suffix}")


def _tag(cfg: Config) -> str:
    return _LANG_TAG[classify_language(cfg.target_language)]


# ----------------------------------------------------------------------
def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    cfg = Config()
    if args.config:
        cfg = cfg.merged(**load_config_file(args.config))
    if args.glossary:
        cfg = cfg.merged(glossary={**cfg.glossary, **load_glossary(args.glossary)})

    image_mode = "skip" if args.no_images else args.image_mode
    cfg = cfg.merged(
        target_language=args.target_language,
        source_language=args.source_language,
        instructions=_read_instructions(args),
        model=args.model,
        effort=args.effort,
        image_effort=args.image_effort,
        image_mode=image_mode,
        image_min_side=args.image_min_side,
        image_debug_dir=args.image_debug_dir,
        font_path=args.font,
        pdf_font_file=args.pdf_font_file,
        concurrency=args.concurrency,
        max_chars_per_batch=args.max_chars_per_batch,
        cache_path=args.cache_path,
        use_cache=False if args.no_cache else None,
        dry_run=True if args.dry_run else None,
        verbose=True if args.verbose else None,
    )
    if cfg.image_mode == "skip":
        cfg = cfg.merged(translate_images=False)

    inputs: List[Path] = []
    for raw in args.inputs:
        path = Path(raw)
        if not path.exists():
            print(f"錯誤：找不到檔案 {path}", file=sys.stderr)
            return 2
        if path.suffix.lower() not in SUPPORTED_SUFFIXES:
            print(f"錯誤：目前只支援 {', '.join(sorted(SUPPORTED_SUFFIXES))}，"
                  f"不支援 {path.suffix or path.name}", file=sys.stderr)
            return 2
        inputs.append(path)

    interactive = sys.stdin.isatty() and not args.yes and not cfg.dry_run
    if interactive:
        cfg = _interactive(cfg, inputs)

    if cfg.cache_path is None:
        cfg = cfg.merged(cache_path=default_cache_path())

    font_path = find_font(cfg.target_language, cfg.font_path)
    if cfg.translate_images and font_path is None:
        print("警告：找不到可用的字型，圖片內的譯文可能畫不出來；"
              "請用 --font 指定一個 TTF/OTF。", file=sys.stderr)
    book = FontBook(font_path)

    print(f"目標語言：{cfg.target_language}　模型：{cfg.model}"
          f"　圖片：{'重繪譯文' if cfg.translate_images else '不處理'}")
    print(f"特殊要求：{cfg.instructions or '（未指定，由 Claude 自行判斷）'}")
    if cfg.glossary:
        print(f"術語表：{len(cfg.glossary)} 條")
    print()

    namespace = TranslationCache.make_namespace(
        cfg.model, cfg.target_language, cfg.source_language,
        cfg.instructions, cfg.glossary,
    )
    cache = TranslationCache(cfg.cache_path, namespace, enabled=cfg.use_cache)

    def log(message: str) -> None:
        if cfg.verbose:
            print(message)

    translator = Translator(cfg, cache=cache, log=log)
    stats: List[FileStats] = []
    failed = False

    from .docx_backend import translate_docx
    from .pdf_backend import translate_pdf

    try:
        for path in inputs:
            out_path = _resolve_output(path, cfg, args.output, len(inputs) > 1)
            if out_path.resolve() == path.resolve():
                print(f"錯誤：輸出會覆蓋輸入檔 {path}", file=sys.stderr)
                return 2
            print(f"▶ {path} → {out_path}")
            handler = translate_pdf if path.suffix.lower() == ".pdf" else translate_docx
            try:
                stat = handler(str(path), str(out_path), translator, cfg, book, log)
            except Exception as exc:                       # noqa: BLE001
                failed = True
                stats.append(FileStats(path=str(path), error=f"{type(exc).__name__}: {exc}"))
                if cfg.verbose:
                    import traceback
                    traceback.print_exc()
                continue
            stats.append(stat)
    except KeyboardInterrupt:
        print("\n已中止。已經翻好的片段留在快取裡，下次會直接沿用。", file=sys.stderr)
        return 130
    finally:
        cache.close()

    print(summarize(cfg, translator.usage, stats, cache.hits))
    return 1 if failed else 0
