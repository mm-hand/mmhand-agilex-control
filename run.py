from __future__ import annotations
import os
import signal
import logging
import sys
from pathlib import Path
from config import ControlConfig
def _logger() -> logging.Logger:
    logger = logging.getLogger("analytic_control")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(handler)
    logger.propagate = False
    return logger
def main() -> int:
    os.chdir(Path(__file__).resolve().parent)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    from control_loop import run_control
    config = ControlConfig()
    logger = _logger()
    logger.info("starting analytic_control mode=%s", config.runtime.mode)
    return run_control(config, logger)
if __name__ == "__main__":
    raise SystemExit(main())
