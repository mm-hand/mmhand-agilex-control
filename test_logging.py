from __future__ import annotations

import csv
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from config import LogConfig
from run_log import SessionLogger


class DriverStatus:
    _fields_ = ("enabled",)
    enabled = True


def test_session_log_records_control_and_diagnostics():
    messages = []
    logger = SimpleNamespace(info=lambda *args: messages.append(args))
    with tempfile.TemporaryDirectory() as directory:
        session = SessionLogger(
            LogConfig(directory=directory, flush_period_s=0.0),
            "dual_sim",
        )
        path = Path(session.telemetry_path)
        session.event("startup_move", "left", command_q=np.zeros(7))
        session.record(
            "teleop",
            "left",
            connected=True,
            tracked=True,
            current_q=np.arange(7),
            command_q=np.ones(7),
            diagnostics={"driver": DriverStatus()},
        )
        session.close(logger)

        rows = list(csv.DictReader(path.open(encoding="utf-8")))
        assert [row["phase"] for row in rows] == ["startup_move", "teleop"]
        assert json.loads(rows[1]["current_q"]) == list(range(7))
        assert json.loads(rows[1]["diagnostics"])["driver"]["enabled"] is True
        assert messages[-1][0] == "final %s current_q=%s command_q=%s"


if __name__ == "__main__":
    test_session_log_records_control_and_diagnostics()
    print("All logging tests passed.")
