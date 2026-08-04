from __future__ import annotations
import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any
import numpy as np
from config import LogConfig
from transforms import quat_xyzw_from_rotation_matrix

SIGNALS = (
    "current_q",
    "ik_q",
    "filtered_q",
    "command_q",
    "pico_raw",
    "pico_filtered",
    "target_flange",
    "current_flange",
    "ik_flange",
    "command_flange",
    "diagnostics",
)
FIELDS = ("wall_time", "elapsed_s", "phase", "side", "connected", "tracked", *SIGNALS)

def _value(value: Any) -> Any:
    if value is None:
        return None
    if hasattr(value, "xyz_m") and hasattr(value, "quat_xyzw"):
        return [*map(float, value.xyz_m), *map(float, value.quat_xyzw)]
    if isinstance(value, dict):
        return {key: _value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_value(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (str, int, float, bool)):
        return value
    fields = getattr(value, "_fields_", ())
    if fields:
        return {field: _value(getattr(value, field, None)) for field in fields}
    array = np.asarray(value, dtype=float)
    if array.shape == (4, 4):
        quat = quat_xyzw_from_rotation_matrix(array[:3, :3])
        return [*map(float, array[:3, 3]), *map(float, quat)]
    return array.tolist()

class SessionLogger:
    def __init__(self, config: LogConfig, mode: str) -> None:
        self.started = time.monotonic()
        self.flush_period_s = max(0.0, config.flush_period_s)
        self.last_flush = self.started
        self.last: dict[str, dict[str, Any]] = {}
        self.sample_count = 0
        self.directory: Path | None = None
        self.text_log_path: Path | None = None
        self.telemetry_path: Path | None = None
        self._file = None
        self._writer = None
        if not config.enabled:
            return
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        self.directory = Path(config.directory) / f"{stamp}_{mode}"
        self.directory.mkdir(parents=True)
        self.text_log_path = self.directory / "control.log"
        self.telemetry_path = self.directory / "telemetry.csv"
        self._file = self.telemetry_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()

    def event(self, phase: str, side: str = "", **signals: Any) -> None:
        self.record(phase, side, connected=None, tracked=None, **signals)

    def record(
        self,
        phase: str,
        side: str,
        *,
        connected: bool | None,
        tracked: bool | None,
        **signals: Any,
    ) -> None:
        now = time.monotonic()
        self.sample_count += 1
        if side:
            self.last[side] = dict(signals)
        if self._writer is None:
            return
        row: dict[str, Any] = {
            "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "elapsed_s": now - self.started,
            "phase": phase,
            "side": side,
            "connected": "" if connected is None else int(connected),
            "tracked": "" if tracked is None else int(tracked),
        }
        for name in SIGNALS:
            value = _value(signals.get(name))
            row[name] = "" if value is None else json.dumps(value, separators=(",", ":"))
        self._writer.writerow(row)
        if now - self.last_flush >= self.flush_period_s:
            self._file.flush()
            self.last_flush = now

    def close(self, logger) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
        logger.info(
            "run summary duration=%.2fs samples=%d telemetry=%s",
            time.monotonic() - self.started,
            self.sample_count,
            self.telemetry_path or "disabled",
        )
        for side, sample in self.last.items():
            logger.info(
                "final %s current_q=%s command_q=%s",
                side,
                sample.get("current_q"),
                sample.get("command_q"),
            )
