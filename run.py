from __future__ import annotations

import os
import signal
from pathlib import Path

from config import ControlConfig
from logging_utils import configure_logging


def main() -> int:
    os.chdir(Path(__file__).resolve().parent)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    from control_loop import run_control

    config = ControlConfig()
    logger = configure_logging(config.log)
    logger.info("starting analytic_control mode=%s", config.runtime.mode)
    return run_control(config, logger)


if __name__ == "__main__":
    raise SystemExit(main())
