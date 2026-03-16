"""
telemetry/watchdog.py
─────────────────────────────────────────────────────────────────────────────
Software watchdog for every ROV module.

Each module registers itself and calls feed() periodically.
The watchdog thread checks all modules every second.
If a module misses its deadline → health status → "dead" → failsafe callback.

Usage
─────
    wdog = Watchdog()
    wdog.register("vision",   timeout_s=1.0)
    wdog.register("ibus",     timeout_s=0.5, critical=True)
    wdog.start(on_critical_failure=emergency_stop)

    # In each module's loop:
    wdog.feed("vision")
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from rov_logger import get_logger

log = get_logger("watchdog")


@dataclass
class ModuleHealth:
    name:         str
    timeout_s:    float
    critical:     bool          # if True and dead → trigger global failsafe
    last_feed:    float = field(default_factory=time.monotonic)
    status:       str = "ok"    # ok | degraded | dead
    error_count:  int = 0
    start_time:   float = field(default_factory=time.monotonic)
    miss_count:   int = 0       # consecutive missed feeds

    @property
    def uptime_s(self) -> float:
        return time.monotonic() - self.start_time

    @property
    def age_s(self) -> float:
        return time.monotonic() - self.last_feed


class Watchdog:

    CHECK_INTERVAL = 0.25   # seconds between watchdog checks

    def __init__(self):
        self._modules:  dict[str, ModuleHealth] = {}
        self._lock      = threading.Lock()
        self._thread:   Optional[threading.Thread] = None
        self._running   = False
        self._on_critical: Optional[Callable] = None
        self._failsafe_fired = False

    # ── Registration ──────────────────────────────────────────────────────

    def register(self, name: str, timeout_s: float = 2.0,
                 critical: bool = False) -> None:
        with self._lock:
            self._modules[name] = ModuleHealth(
                name=name, timeout_s=timeout_s, critical=critical
            )
        log.info(f"Watchdog: registered '{name}' timeout={timeout_s}s critical={critical}")

    def feed(self, name: str) -> None:
        with self._lock:
            if name in self._modules:
                m = self._modules[name]
                m.last_feed   = time.monotonic()
                m.miss_count  = 0
                if m.status == "dead":
                    log.info(f"Watchdog: '{name}' recovered from dead")
                m.status = "ok"

    def increment_error(self, name: str) -> None:
        with self._lock:
            if name in self._modules:
                self._modules[name].error_count += 1

    def set_degraded(self, name: str, reason: str = "") -> None:
        with self._lock:
            if name in self._modules:
                self._modules[name].status = "degraded"
        log.warning(f"Watchdog: '{name}' degraded — {reason}")

    def get_status(self) -> dict:
        with self._lock:
            return {
                n: {
                    "status":       m.status,
                    "uptime_s":     round(m.uptime_s, 1),
                    "age_s":        round(m.age_s, 2),
                    "error_count":  m.error_count,
                }
                for n, m in self._modules.items()
            }

    def all_ok(self) -> bool:
        with self._lock:
            return all(m.status == "ok" for m in self._modules.values())

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self, on_critical_failure: Optional[Callable] = None) -> None:
        self._on_critical = on_critical_failure
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="watchdog"
        )
        self._thread.start()
        log.info("Watchdog started")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    # ── Internal ──────────────────────────────────────────────────────────

    def _loop(self) -> None:
        while self._running:
            time.sleep(self.CHECK_INTERVAL)
            with self._lock:
                modules = list(self._modules.values())

            critical_dead = []
            for m in modules:
                if m.age_s > m.timeout_s:
                    m.miss_count += 1
                    prev_status = m.status
                    m.status = "dead" if m.miss_count >= 3 else "degraded"

                    if m.status != prev_status:
                        lvl = log.critical if m.status == "dead" and m.critical else log.warning
                        lvl(
                            f"Watchdog: '{m.name}' is {m.status} "
                            f"(age={m.age_s:.2f}s timeout={m.timeout_s}s "
                            f"errors={m.error_count})"
                        )

                    if m.status == "dead" and m.critical:
                        critical_dead.append(m.name)

            if critical_dead and not self._failsafe_fired:
                self._failsafe_fired = True
                log.critical(
                    f"CRITICAL MODULES DEAD: {critical_dead} — firing failsafe"
                )
                if self._on_critical:
                    try:
                        self._on_critical()
                    except Exception as e:
                        log.critical(f"Failsafe callback raised: {e}")
