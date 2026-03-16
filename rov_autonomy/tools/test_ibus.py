"""
tools/test_ibus.py
Bench test for iBus link — run this before any pool session to verify
the ESP32 is receiving commands correctly.

What it does:
  1. Arms the ESP32
  2. Sweeps surge forward/backward
  3. Sweeps yaw left/right
  4. Disarms

Watch the ESP32 serial output (115200 baud) — you should see:
  - mode changing to PITCH_STAB
  - armed = true
  - cmdSurgeOut changing as we sweep

Safety: Run with PROPELLERS REMOVED or thrusters out of water.
"""

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from control.ibus_interface import IBusInterface


def ramp(ibus, channel_fn, low, high, steps=20, hold=0.1):
    for i in range(steps + 1):
        v = low + (high - low) * i / steps
        channel_fn(v)
        time.sleep(hold)
    for i in range(steps + 1):
        v = high - (high - low) * i / steps
        channel_fn(v)
        time.sleep(hold)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
    ibus = IBusInterface(port=port, rate_hz=50)
    ibus.start(armed=False)

    print("Waiting 3s for ESP32 to lock onto iBus signal...")
    time.sleep(3.0)

    print("Step 1: ARM")
    ibus.arm()
    time.sleep(1.0)

    print("Step 2: Surge sweep (forward → neutral → reverse → neutral)")
    ramp(ibus, lambda v: ibus.set_motion(surge=v), 0.0, 0.4)
    time.sleep(0.5)
    ramp(ibus, lambda v: ibus.set_motion(surge=v), 0.0, -0.3)
    time.sleep(0.5)

    print("Step 3: Yaw sweep")
    ramp(ibus, lambda v: ibus.set_motion(yaw=v), 0.0, 0.5)
    time.sleep(0.5)
    ramp(ibus, lambda v: ibus.set_motion(yaw=v), 0.0, -0.5)
    time.sleep(0.5)

    print("Step 4: Heave sweep")
    ramp(ibus, lambda v: ibus.set_motion(heave=v), 0.0, 0.4)
    time.sleep(0.5)

    print("Step 5: DISARM")
    ibus.stop_motion()
    time.sleep(0.5)
    ibus.disarm()
    time.sleep(1.0)
    ibus.stop()
    print("Test complete.")


if __name__ == "__main__":
    main()
