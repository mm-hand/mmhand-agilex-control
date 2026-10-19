from __future__ import annotations

import threading

from logging_utils import LogBuffer


def test_log_buffer_returns_only_new_lines():
    buffer = LogBuffer()
    buffer.append("control", "first")
    buffer.append("pico", "second")
    assert len(buffer.since(0)) == 2
    sequence = buffer.sequence
    buffer.append("control", "third")
    lines, next_sequence = buffer.read_since(sequence)
    assert len(lines) == 1
    assert "third" in lines[0]
    assert next_sequence == sequence + 1


def test_log_buffer_is_bounded_and_thread_safe():
    buffer = LogBuffer(max_lines=10)
    threads = [threading.Thread(target=lambda index=index: buffer.append("test", f"line-{index}")) for index in range(100)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    lines = buffer.since(0)
    assert len(lines) == 10
    assert buffer.sequence == 100
