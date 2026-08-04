from __future__ import annotations
import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
from config import LogConfig

def configure_logging(config: LogConfig, path: Path | None) -> logging.Logger:
    logger = logging.getLogger("analytic_control")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    logger.propagate = False
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    stream = logging.StreamHandler(sys.stdout)
    stream.setFormatter(formatter)
    logger.addHandler(stream)
    if config.enabled and path is not None:
        file_handler = RotatingFileHandler(
            path,
            maxBytes=config.max_bytes,
            backupCount=config.backup_count,
            encoding="utf-8",
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    return logger
