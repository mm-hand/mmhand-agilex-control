from __future__ import annotations

import threading

from config import LogConfig
from logging_utils import LogBuffer, configure_logging


def test_log_buffer_returns_only_new_lines():
    buffer = LogBuffer()
    buffer.append("control", "first")
    buffer.append("pico", "second")
    assert len(buffer.read_since(0)[0]) == 2
    sequence = buffer.read_since(0)[1]
    buffer.append("control", "third")
    lines, next_sequence = buffer.read_since(sequence)
    assert len(lines) == 1
    assert "third" in lines[0]
    assert next_sequence == sequence + 1


def test_log_buffer_is_bounded_and_thread_safe():
    buffer = LogBuffer(max_lines=10)
    threads = [
        threading.Thread(target=lambda index=index: buffer.append("test", f"line-{index}"))
        for index in range(100)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(buffer.read_since(0)[0]) == 10


def test_configure_logging_writes_file_and_buffer(tmp_path):
    buffer = LogBuffer()
    logger = configure_logging(LogConfig(directory=str(tmp_path)), buffer)
    logger.info("hello")
    for handler in logger.handlers:
        handler.flush()
    assert "hello" in (tmp_path / "control.log").read_text(encoding="utf-8")
    assert any("hello" in line for line in buffer.read_since(0)[0])
