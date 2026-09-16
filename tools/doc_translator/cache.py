"""譯文快取。

同一份文件重跑、或多份文件共用同樣的句子時，直接命中快取而不再呼叫 API。
key 由「模型 + 目標語言 + 特殊要求 + 術語表 + 原文」一起雜湊而成，
因此改了特殊要求就一定會重譯，不會拿到舊要求下的譯文。
"""

from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
from pathlib import Path
from typing import Optional


class TranslationCache:
    def __init__(self, path: Optional[str], namespace: str, enabled: bool = True):
        self.enabled = bool(enabled and path)
        self._ns = namespace
        self._lock = threading.Lock()
        self._conn: Optional[sqlite3.Connection] = None
        self.hits = 0
        if not self.enabled:
            return
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.execute(
            "CREATE TABLE IF NOT EXISTS entries ("
            " key TEXT PRIMARY KEY, value TEXT NOT NULL,"
            " created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP)"
        )
        self._conn.commit()

    # ------------------------------------------------------------------
    @staticmethod
    def make_namespace(model: str, target: str, source: str, instructions: str,
                       glossary: dict) -> str:
        payload = json.dumps(
            {"m": model, "t": target, "s": source, "i": instructions,
             "g": sorted(glossary.items())},
            ensure_ascii=False, sort_keys=True,
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]

    def _key(self, kind: str, content: str) -> str:
        digest = hashlib.sha256(content.encode("utf-8")).hexdigest()
        return f"{self._ns}:{kind}:{digest}"

    # ------------------------------------------------------------------
    def get(self, kind: str, content: str):
        if not self.enabled:
            return None
        key = self._key(kind, content)
        with self._lock:
            row = self._conn.execute(
                "SELECT value FROM entries WHERE key = ?", (key,)
            ).fetchone()
        if row is None:
            return None
        self.hits += 1
        return json.loads(row[0])

    def put(self, kind: str, content: str, value) -> None:
        if not self.enabled:
            return
        key = self._key(kind, content)
        blob = json.dumps(value, ensure_ascii=False)
        with self._lock:
            self._conn.execute(
                "INSERT OR REPLACE INTO entries (key, value) VALUES (?, ?)", (key, blob)
            )
            self._conn.commit()

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None
