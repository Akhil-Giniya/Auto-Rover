#!/usr/bin/env python3
"""
tools/test_ibus.py — Bench test: verify ESP32 receives iBus commands.
Run with PROPELLERS REMOVED or thrusters out of water.

Usage:
  python tools/test_ibus.py
  python tools/test_ibus.py --port /dev/ttyS0
"""

import sys, os, time, argparse
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hw_interface.ibus_interface import IBusInterface


def ramp(ibus, fn, lo, hi, steps=20, hold=0.08):
    for i in range(steps + 1):
        fn(lo + (hi - lo) * i / steps)
        time.sleep(hold)
    for i in range(steps + 1):
        fn(hi - (hi - lo) * i / steps)
        time.sleep(hold)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial0")
    args = ap.parse_args()

    ibus = IBusInterface(port=args.port)
    ibus.start(armed=False)

    print("Waiting 3 s for ESP32 iBus lock...")
    time.sleep(3.0)

    print("Step 1: ARM")
    ibus.arm(); time.sleep(1.0)

    print("Step 2: Surge sweep")
    ramp(ibus, lambda v: ibus.set_motion(surge=v), 0.0, 0.4)
    time.sleep(0.5)
    ramp(ibus, lambda v: ibus.set_motion(surge=v), 0.0, -0.3)
    time.sleep(0.5)

    print("Step 3: Yaw sweep")
    ramp(ibus, lambda v: ibus.set_motion(yaw=v), 0.0, 0.5)
    time.sleep(0.3)
    ramp(ibus, lambda v: ibus.set_motion(yaw=v), 0.0, -0.5)
    time.sleep(0.5)

    print("Step 4: Heave sweep")
    ramp(ibus, lambda v: ibus.set_motion(heave=v), 0.0, 0.4)
    time.sleep(0.5)

    print("Step 5: Ball drop servo test")
    ibus.set_ball_drop(True); time.sleep(1.5)
    ibus.set_ball_drop(False); time.sleep(0.5)

    print("Step 6: DISARM")
    ibus.stop_motion(); time.sleep(0.3)
    ibus.disarm(); time.sleep(1.0)
    ibus.stop()

    print(f"\nDone. packets_sent={ibus.packets_sent} errors={ibus.send_errors}")


if __name__ == "__main__":
    main()
