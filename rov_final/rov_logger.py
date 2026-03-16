"""
rov_logger.py
─────────────────────────────────────────────────────────────────────────────
Central logging factory for the entire ROV codebase.

Every module calls:
    from rov_logger import get_logger
    log = get_logger("vision")

Features
--------
• Rotating file logs:  logs/<module>_<date>.log  (5 MB × 5 backups)
• Console output with colour-coded level names
• Machine-readable JSON side-log:  logs/telemetry.jsonl  (for post-run analysis)
• Thread-safe
• CRITICAL errors also appended to logs/CRITICAL.log (never rotated)

Log levels
----------
DEBUG    → verbose: every frame result, every PID tick
INFO     → state transitions, detections, commands sent
WARNING  → recoverable issues: sensor timeout, missed frame
ERROR    → non-fatal failures: serial retry, detector exception
CRITICAL → fatal: iBus link lost, camera hardware failure
"""

import logging
import logging.handlers
import os
import sys
import json
import threading
import datetime
from pathlib import Path

LOG_DIR = Path(__file__).parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

_loggers: dict[str, logging.Logger] = {}
_lock = threading.Lock()

# ── ANSI colour codes ──────────────────────────────────────────────────────
_COLOURS = {
    "DEBUG":    "\033[36m",   # cyan
    "INFO":     "\033[32m",   # green
    "WARNING":  "\033[33m",   # yellow
    "ERROR":    "\033[31m",   # red
    "CRITICAL": "\033[35m",   # magenta
}
_RESET = "\033[0m"


class _ColourFormatter(logging.Formatter):
    FMT = "%(asctime)s %(colour)s[%(levelname)-8s]%(reset)s [%(name)s] %(message)s"

    def format(self, record):
        record.colour = _COLOURS.get(record.levelname, "")
        record.reset  = _RESET
        return super().format(record)


class _JSONHandler(logging.FileHandler):
    """Appends one JSON object per log line to telemetry.jsonl."""

    def emit(self, record):
        try:
            entry = {
                "ts":      datetime.datetime.utcnow().isoformat(),
                "level":   record.levelname,
                "module":  record.name,
                "msg":     record.getMessage(),
            }
            if record.exc_info:
                entry["exc"] = self.formatException(record.exc_info)
            self.stream.write(json.dumps(entry) + "\n")
            self.flush()
        except Exception:
            self.handleError(record)


def get_logger(name: str, level: int = logging.DEBUG) -> logging.Logger:
    """Return (or create) a named logger wired to file + console + JSON."""
    with _lock:
        if name in _loggers:
            return _loggers[name]

        log = logging.getLogger(f"rov.{name}")
        log.setLevel(level)
        log.propagate = False

        # ── rotating file handler ──────────────────────────────────────────
        today = datetime.date.today().isoformat()
        file_handler = logging.handlers.RotatingFileHandler(
            LOG_DIR / f"{name}_{today}.log",
            maxBytes=5 * 1024 * 1024,   # 5 MB
            backupCount=5,
            encoding="utf-8"
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)-8s] [%(name)s] %(message)s"
        ))

        # ── console handler ────────────────────────────────────────────────
        console = logging.StreamHandler(sys.stdout)
        console.setLevel(logging.INFO)
        console.setFormatter(_ColourFormatter(
            "%(asctime)s %(colour)s[%(levelname)-8s]%(reset)s [%(name)s] %(message)s"
        ))

        # ── JSON telemetry handler ─────────────────────────────────────────
        json_handler = _JSONHandler(LOG_DIR / "telemetry.jsonl", encoding="utf-8")
        json_handler.setLevel(logging.INFO)

        # ── CRITICAL-only permanent log ────────────────────────────────────
        critical_handler = logging.FileHandler(
            LOG_DIR / "CRITICAL.log", encoding="utf-8"
        )
        critical_handler.setLevel(logging.CRITICAL)
        critical_handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] [%(name)s] %(message)s"
        ))

        for h in (file_handler, console, json_handler, critical_handler):
            log.addHandler(h)

        _loggers[name] = log
        return log
