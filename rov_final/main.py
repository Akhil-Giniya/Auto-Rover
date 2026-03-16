"""
main.py
─────────────────────────────────────────────────────────────────────────────
ROV Autonomy — shippable entry point.

Startup sequence
────────────────
1. Load config
2. Start telemetry broker (ZMQ)
3. Register all modules with watchdog
4. Start iBus interface → wait for ESP32 link
5. Start vision pipeline → wait for first frame
6. Arm + 2 s warm-up
7. Run mission state machine
8. Graceful shutdown

Usage
─────
  python main.py                      # full autonomous mission
  python main.py --dry-run            # vision + logic, NO iBus output
  python main.py --debug              # show OpenCV window
  python main.py --port /dev/ttyS0   # custom UART port
  python main.py --no-telemetry       # disable ZMQ bus (saves memory)
"""

import argparse
import json
import os
import sys
import signal
import time
import threading
from pathlib import Path

# ── ensure rov_final is on path ───────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))

from rov_logger import get_logger
from telemetry.bus import TelemetryBroker, TelemetryPublisher, HealthMsg
from telemetry.watchdog import Watchdog
from hw_interface.ibus_interface import IBusInterface, IBusError
from vision.pipeline import VisionPipeline
from autonomy_core.state_machine import MissionStateMachine

log = get_logger("main")

# ── Global handles for signal handler ─────────────────────────────────────
_ibus:    IBusInterface | None   = None
_vision:  VisionPipeline | None  = None
_sm:      MissionStateMachine | None = None
_wdog:    Watchdog | None        = None
_shutdown_event = threading.Event()


def _signal_handler(sig, frame):
    log.warning(f"Signal {sig} received — initiating graceful shutdown")
    _shutdown_event.set()
    if _sm:
        _sm.stop()


def load_config(path: str) -> dict:
    try:
        with open(path) as f:
            cfg = json.load(f)
        log.info(f"Config loaded: {path}")
        return cfg
    except FileNotFoundError:
        log.critical(f"Config file not found: {path}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        log.critical(f"Config JSON parse error: {e}")
        sys.exit(1)


def make_dry_run_ibus(ibus: IBusInterface) -> IBusInterface:
    """Wrap iBus with no-op methods for dry-run mode."""
    def _noop(*a, **kw): pass
    ibus.set_motion  = _noop
    ibus.stop_motion = _noop
    ibus.arm         = _noop
    ibus.disarm      = _noop
    ibus.set_ball_drop = _noop
    ibus.emergency_stop = _noop
    ibus.start = lambda armed=False: log.info("[dry-run] iBus start (no-op)")
    ibus.stop  = lambda: log.info("[dry-run] iBus stop (no-op)")
    return ibus


def health_reporter(pub: TelemetryPublisher, wdog: Watchdog,
                    ibus: IBusInterface, vision: VisionPipeline,
                    stop_event: threading.Event) -> None:
    """Background thread: publish health metrics every 5 s."""
    while not stop_event.is_set():
        try:
            status = wdog.get_status()
            for name, info in status.items():
                pub.pub_health(HealthMsg(
                    module=name,
                    status=info["status"],
                    error_count=info["error_count"],
                    uptime_s=info["uptime_s"],
                    ts=time.monotonic()
                ))
            # Log summary
            ok = wdog.all_ok()
            if not ok:
                log.warning(f"Health: {status}")
            log.debug(
                f"iBus: pkts={ibus.packets_sent} errs={ibus.send_errors} "
                f"age={ibus.last_send_age_ms:.0f}ms | "
                f"Vision: fps={vision.fps:.1f} frames={vision.frame_count}"
            )
        except Exception as e:
            log.warning(f"Health reporter error: {e}")
        stop_event.wait(5.0)


def main():
    global _ibus, _vision, _sm, _wdog

    # ── Argument parsing ──────────────────────────────────────────────────
    ap = argparse.ArgumentParser(description="ROV Autonomy System")
    ap.add_argument("--port",          default="/dev/serial0")
    ap.add_argument("--config",        default="config.json")
    ap.add_argument("--dry-run",       action="store_true")
    ap.add_argument("--debug",         action="store_true")
    ap.add_argument("--no-telemetry",  action="store_true")
    ap.add_argument("--log-level",     default="INFO",
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()

    # ── Signal handlers ───────────────────────────────────────────────────
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    log.info("=" * 60)
    log.info("ROV AUTONOMY SYSTEM — STARTUP")
    log.info(f"  dry-run:      {args.dry_run}")
    log.info(f"  debug:        {args.debug}")
    log.info(f"  port:         {args.port}")
    log.info(f"  config:       {args.config}")
    log.info(f"  telemetry:    {not args.no_telemetry}")
    log.info("=" * 60)

    # ── Config ────────────────────────────────────────────────────────────
    cfg = load_config(args.config)

    # ── Telemetry broker ──────────────────────────────────────────────────
    broker = TelemetryBroker()
    pub    = TelemetryPublisher("main")
    if not args.no_telemetry:
        broker.start()
        time.sleep(0.2)   # ZMQ bind settling

    # ── Watchdog ──────────────────────────────────────────────────────────
    _wdog = Watchdog()
    _wdog.register("ibus",      timeout_s=0.6,  critical=True)
    _wdog.register("vision",    timeout_s=2.0,  critical=True)
    _wdog.register("autonomy",  timeout_s=2.0,  critical=True)

    def _emergency_stop():
        log.critical("WATCHDOG EMERGENCY STOP")
        if _ibus:
            _ibus.emergency_stop()
        _shutdown_event.set()

    _wdog.start(on_critical_failure=_emergency_stop)

    # ── iBus interface ────────────────────────────────────────────────────
    _ibus = IBusInterface(port=args.port, baud=115200, watchdog=_wdog)
    if args.dry_run:
        _ibus = make_dry_run_ibus(_ibus)

    try:
        _ibus.start(armed=False)
    except IBusError as e:
        log.critical(f"iBus startup failed: {e}")
        sys.exit(2)

    log.info("Waiting 2 s for ESP32 to lock onto iBus signal...")
    time.sleep(2.0)

    # ── Vision pipeline ───────────────────────────────────────────────────
    _vision = VisionPipeline(cfg, watchdog=_wdog,
                              publisher=pub if not args.no_telemetry else None)
    _vision.start()

    if not _vision.frame_count and not args.dry_run:
        log.critical("Vision pipeline produced no frames — aborting")
        _ibus.stop()
        sys.exit(3)

    # ── Health reporter thread ────────────────────────────────────────────
    health_thread = threading.Thread(
        target=health_reporter,
        args=(pub, _wdog, _ibus, _vision, _shutdown_event),
        daemon=True, name="health"
    )
    health_thread.start()

    # ── Arm and run ───────────────────────────────────────────────────────
    log.info("Pre-arm checks passed — arming in 1 s")
    time.sleep(1.0)
    _ibus.arm()
    time.sleep(1.0)   # Give ESP32 time to see arm command

    _sm = MissionStateMachine(
        ibus=_ibus,
        vision=_vision,
        cfg=cfg,
        watchdog=_wdog,
        publisher=pub if not args.no_telemetry else None,
        debug_display=args.debug
    )

    # Run mission in main thread
    try:
        _sm.run()
    except Exception as e:
        log.critical(f"Mission crashed: {e}", exc_info=True)
    finally:
        _shutdown()


def _shutdown():
    log.info("Shutting down all systems...")
    _shutdown_event.set()
    if _sm:
        try: _sm.stop()
        except Exception: pass
    if _ibus:
        try: _ibus.stop()
        except Exception: pass
    if _vision:
        try: _vision.stop()
        except Exception: pass
    if _wdog:
        try: _wdog.stop()
        except Exception: pass
    log.info("Shutdown complete. Goodbye.")


if __name__ == "__main__":
    main()
