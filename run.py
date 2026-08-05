from __future__ import annotations

import os
import signal
from pathlib import Path

from config import ControlConfig
from diagnostics import RunDiagnostics
from logging_utils import configure_logging


def main() -> int:
    os.chdir(Path(__file__).resolve().parent)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    config = ControlConfig()
    logger = configure_logging(config.log)
    diagnostics = RunDiagnostics(config)
    logger.info("starting analytic_control mode=%s", config.runtime.mode)
    try:
        from control_loop import run_control

        return run_control(config, logger, diagnostics)
    except BaseException as exc:
        diagnostics.record("error", event="exception", note=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        diagnostics.close(logger)


if __name__ == "__main__":
    raise SystemExit(main())
