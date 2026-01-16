"""
Parsed data logger.

PDF says: "Сохраняться должны уже распарсенные данные в формате .log"
We implement JSON Lines (.log): one JSON object per line.
"""

from __future__ import annotations
import json
from dataclasses import asdict, is_dataclass
from typing import Optional, Any
import time

class ParsedLogger:
    def __init__(self) -> None:
        self._fh: Optional[object] = None
        self.path: str = ""

    @property
    def is_running(self) -> bool:
        return self._fh is not None

    def start(self, path: str) -> None:
        self.stop()
        self.path = path
        self._fh = open(path, "a", encoding="utf-8")
        # Header marker (optional)
        self._fh.write(self.format_line({"_type": "log_start", "unix_time": time.time()}) + "\n")
        self._fh.flush()

    def stop(self) -> None:
        if self._fh:
            try:
                self._fh.write(self.format_line({"_type": "log_stop", "unix_time": time.time()}) + "\n")
                self._fh.flush()
                self._fh.close()
            finally:
                self._fh = None

    def write(self, obj: Any) -> None:
        if not self._fh:
            return
        line = self.format_line(obj)
        self.write_line(line)

    def write_line(self, line: str) -> None:
        if not self._fh:
            return
        self._fh.write(line + "\n")
        self._fh.flush()

    def format_line(self, obj: Any) -> str:
        if is_dataclass(obj):
            payload = asdict(obj)
            payload["_type"] = obj.__class__.__name__
        elif isinstance(obj, dict):
            payload = obj
        else:
            payload = {"_type": type(obj).__name__, "value": repr(obj)}
        return json.dumps(payload, ensure_ascii=False)
