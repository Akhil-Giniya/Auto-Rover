"""
tools/tune_hsv.py
Interactive HSV tuner — run this on the RPi with a USB display or VNC
to tune color thresholds in the actual pool under competition lighting.

Usage:
  python tools/tune_hsv.py --target orange_flare
  python tools/tune_hsv.py --target blue_drum
  python tools/tune_hsv.py --target red_drum
  python tools/tune_hsv.py --target green_mat

Controls:
  s  — save current values back to config.json
  q  — quit without saving
"""

import argparse
import json
import sys
import os
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def nothing(_): pass


TARGETS = ["orange_flare", "blue_drum", "green_mat"]   # red_drum needs 2 ranges


def tune(target: str, config_path: str = "config.json"):
    with open(config_path) as f:
        cfg = json.load(f)

    if target == "red_drum":
        print("Red drum uses two HSV ranges. Edit config.json manually after tuning.")
        t_cfg = cfg["red_drum"]["ranges"][0]
    else:
        t_cfg = cfg[target]

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H low",  "HSV Tuner", t_cfg.get("h_low", 0),   180, nothing)
    cv2.createTrackbar("H high", "HSV Tuner", t_cfg.get("h_high", 180), 180, nothing)
    cv2.createTrackbar("S low",  "HSV Tuner", t_cfg.get("s_low", 0),   255, nothing)
    cv2.createTrackbar("S high", "HSV Tuner", t_cfg.get("s_high", 255), 255, nothing)
    cv2.createTrackbar("V low",  "HSV Tuner", t_cfg.get("v_low", 0),   255, nothing)
    cv2.createTrackbar("V high", "HSV Tuner", t_cfg.get("v_high", 255), 255, nothing)

    print(f"Tuning: {target}  |  s=save  q=quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h_l  = cv2.getTrackbarPos("H low",  "HSV Tuner")
        h_h  = cv2.getTrackbarPos("H high", "HSV Tuner")
        s_l  = cv2.getTrackbarPos("S low",  "HSV Tuner")
        s_h  = cv2.getTrackbarPos("S high", "HSV Tuner")
        v_l  = cv2.getTrackbarPos("V low",  "HSV Tuner")
        v_h  = cv2.getTrackbarPos("V high", "HSV Tuner")

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([h_l, s_l, v_l]),
                           np.array([h_h, s_h, v_h]))
        masked = cv2.bitwise_and(frame, frame, mask=mask)
        display = np.hstack([frame, masked])
        cv2.putText(display, f"{target}  H:[{h_l},{h_h}] S:[{s_l},{s_h}] V:[{v_l},{v_h}]",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow("HSV Tuner", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            t_cfg["h_low"]  = h_l
            t_cfg["h_high"] = h_h
            t_cfg["s_low"]  = s_l
            t_cfg["s_high"] = s_h
            t_cfg["v_low"]  = v_l
            t_cfg["v_high"] = v_h
            with open(config_path, "w") as f:
                json.dump(cfg, f, indent=2)
            print(f"Saved {target} HSV values to {config_path}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="blue_drum",
                        choices=["orange_flare", "blue_drum", "green_mat", "red_drum"])
    parser.add_argument("--config", default="config.json")
    args = parser.parse_args()
    tune(args.target, args.config)
