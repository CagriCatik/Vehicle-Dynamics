"""Centralized logging framework for simulation, control, and validation runs.

Outputs structured, timestamped logs to console and to logs/vehicle_dynamics.log.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path


def get_logs_dir() -> Path:
    """Return the absolute path to the centralized logs/ directory."""
    env_dir = os.getenv("VEHICLE_DYNAMICS_LOGS_DIR")
    if env_dir:
        p = Path(env_dir).expanduser().resolve()
        p.mkdir(parents=True, exist_ok=True)
        return p

    # Look for top-level logs/ in workspace root
    root_logs = Path(__file__).resolve().parents[2] / "logs"
    root_logs.mkdir(parents=True, exist_ok=True)
    return root_logs


def setup_logger(
    name: str = "vehicle_dynamics",
    log_file: str | Path | None = None,
    level: int = logging.INFO,
    console_output: bool = True,
) -> logging.Logger:
    """Configure and return a standardized logger."""
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid duplicate handlers if already configured
    if logger.handlers:
        return logger

    formatter = logging.Formatter(
        "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # File handler writing to logs/
    target_log_file = log_file or (get_logs_dir() / f"{name}.log")
    fh = logging.FileHandler(target_log_file, encoding="utf-8")
    fh.setLevel(level)
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    # Optional console handler
    if console_output:
        ch = logging.StreamHandler()
        ch.setLevel(level)
        ch.setFormatter(formatter)
        logger.addHandler(ch)

    return logger


def get_logger(name: str = "vehicle_dynamics") -> logging.Logger:
    """Get or create logger with default configuration."""
    return setup_logger(name)


__all__ = ("setup_logger", "get_logger", "get_logs_dir")
