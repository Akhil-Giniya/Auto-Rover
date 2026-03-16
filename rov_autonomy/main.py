"""
main.py
ROV autonomy entry point.

Usage:
  python main.py                  # full autonomous mission
  python main.py --debug          # show camera window
  python main.py --dry-run        # camera + detection only, no iBus output
  python main.py --port /dev/ttyS0
"""

import argparse
import json
import logging
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("rov_mission.log")
    ]
)
log = logging.getLogger("main")


def load_config(path: str = "config.json") -> dict:
    with open(path) as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser(description="ROV Autonomy")
    parser.add_argument("--port",    default="/dev/serial0",
                        help="UART port to ESP32")
    parser.add_argument("--debug",   action="store_true",
                        help="Show OpenCV debug window")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run vision only, do NOT send iBus commands")
    parser.add_argument("--config",  default="config.json")
    args = parser.parse_args()

    cfg = load_config(args.config)
    log.info("Config loaded")

    from vision.camera import Camera
    from control.ibus_interface import IBusInterface
    from state_machine import MissionStateMachine

    camera = Camera()
    camera.start()
    log.info("Camera ready")

    ibus = IBusInterface(port=args.port, rate_hz=50)

    if args.dry_run:
        log.info("DRY RUN — iBus disabled")
        # Replace ibus methods with no-ops
        ibus.set_motion   = lambda **kw: None
        ibus.stop_motion  = lambda: None
        ibus.arm          = lambda: None
        ibus.disarm       = lambda: None
        ibus.set_channel  = lambda i, v: None
        ibus.start        = lambda armed=False: log.info("[dry-run] ibus.start()")
        ibus.stop         = lambda: log.info("[dry-run] ibus.stop()")

    try:
        ibus.start(armed=True)
        log.info("iBus started — arming in 2 s")
        time.sleep(2.0)   # Allow ESP32 to lock onto iBus signal

        sm = MissionStateMachine(ibus, camera, cfg, debug_display=args.debug)
        sm.run()

    except KeyboardInterrupt:
        log.info("Interrupted by user")
    finally:
        ibus.stop()
        camera.stop()
        log.info("Shutdown complete")


if __name__ == "__main__":
    main()
