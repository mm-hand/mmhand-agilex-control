from __future__ import annotations

import logging
import sys
import threading
from collections import deque
from datetime import datetime
from logging.handlers import RotatingFileHandler
from pathlib import Path

from config import LogConfig


class LogBuffer:
    def __init__(self, max_lines: int = 5000) -> None:
        self._lock = threading.Lock()
        self._lines: deque[tuple[int, str]] = deque(maxlen=max_lines)
        self._sequence = 0

    def append(self, source: str, message: str) -> None:
        text = message.rstrip("\r\n")
        if not text:
            return
        line = f"{datetime.now().strftime('%H:%M:%S')} [{source}] {text}"
        with self._lock:
            self._sequence += 1
            self._lines.append((self._sequence, line))

    def read_since(self, sequence: int) -> tuple[list[str], int]:
        with self._lock:
            lines = [line for seq, line in self._lines if seq > sequence]
            return lines, self._sequence


class LogBufferHandler(logging.Handler):
    def __init__(self, buffer: LogBuffer, source: str = "control") -> None:
        super().__init__()
        self.buffer = buffer
        self.source = source

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.buffer.append(self.source, record.getMessage())
        except Exception:
            self.handleError(record)


def configure_logging(config: LogConfig, buffer: LogBuffer | None = None) -> logging.Logger:
    logger = logging.getLogger("analytic_control")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    logger.propagate = False
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    stream = logging.StreamHandler(sys.stdout)
    stream.setFormatter(formatter)
    logger.addHandler(stream)
    if buffer is not None:
        logger.addHandler(LogBufferHandler(buffer, "control"))
    if config.enabled:
        try:
            directory = Path(config.directory)
            directory.mkdir(parents=True, exist_ok=True)
            file_handler = RotatingFileHandler(
                directory / "control.log",
                maxBytes=config.max_bytes,
                backupCount=config.backup_count,
                encoding="utf-8",
            )
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
        except OSError as exc:
            logger.warning("file logging unavailable: %s", exc)
    return logger
