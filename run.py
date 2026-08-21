from __future__ import annotations
import os
import signal
from pathlib import Path
from config import ControlConfig
from logging_utils import configure_logging
from run_log import SessionLogger

def main() -> int:
    os.chdir(Path(__file__).resolve().parent)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    from control_loop import run_control
    config = ControlConfig()
    session = SessionLogger(config.log, config.runtime.mode)
    logger = configure_logging(config.log, session.text_log_path)
    logger.info("starting analytic_control mode=%s", config.runtime.mode)
    try:
        return run_control(config, logger, session)
    except Exception:
        logger.exception("analytic_control failed")
        raise
    finally:
        session.close(logger)
if __name__ == "__main__":
    raise SystemExit(main())
