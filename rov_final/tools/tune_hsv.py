#!/usr/bin/env python3
"""
tools/tune_hsv.py — Interactive poolside HSV colour tuner.

Usage:
  python tools/tune_hsv.py --target blue_drum
  python tools/tune_hsv.py --target orange_flare
  python tools/tune_hsv.py --target green_mat
  python tools/tune_hsv.py --target red_drum   (edits first range)

Controls:
  s  — save current sliders back to config.json
  q  — quit without saving
  r  — reset sliders to current config values
"""

import argparse, json, sys, os
import cv2, numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config.json")
TARGETS = ["orange_flare", "blue_drum", "green_mat", "red_drum"]


def nothing(_): pass


def load_cfg():
    with open(CONFIG_PATH) as f:
        return json.load(f)


def save_cfg(cfg):
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"[Saved] {CONFIG_PATH}")


def tune(target: str):
    cfg = load_cfg()
    if target == "red_drum":
        t = cfg["red_drum"]["ranges"][0]
    else:
        t = cfg[target]

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    win = "HSV Tuner — s=save  q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H low",  win, t.get("h_low", 0),   180, nothing)
    cv2.createTrackbar("H high", win, t.get("h_high", 180), 180, nothing)
    cv2.createTrackbar("S low",  win, t.get("s_low", 0),   255, nothing)
    cv2.createTrackbar("S high", win, t.get("s_high", 255), 255, nothing)
    cv2.createTrackbar("V low",  win, t.get("v_low", 0),   255, nothing)
    cv2.createTrackbar("V high", win, t.get("v_high", 255), 255, nothing)

    print(f"Tuning: {target}  |  s=save  q=quit  r=reset")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        hl  = cv2.getTrackbarPos("H low",  win)
        hh  = cv2.getTrackbarPos("H high", win)
        sl  = cv2.getTrackbarPos("S low",  win)
        sh  = cv2.getTrackbarPos("S high", win)
        vl  = cv2.getTrackbarPos("V low",  win)
        vh  = cv2.getTrackbarPos("V high", win)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([hl, sl, vl]), np.array([hh, sh, vh]))
        out  = cv2.bitwise_and(frame, frame, mask=mask)
        info = f"{target}  H:[{hl},{hh}] S:[{sl},{sh}] V:[{vl},{vh}]"
        cv2.putText(out, info, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
        cv2.imshow(win, np.hstack([frame, out]))

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('s'):
            t.update(h_low=hl, h_high=hh, s_low=sl, s_high=sh, v_low=vl, v_high=vh)
            cfg2 = load_cfg()
            if target == "red_drum":
                cfg2["red_drum"]["ranges"][0] = t
            else:
                cfg2[target] = t
            save_cfg(cfg2)
        elif k == ord('r'):
            cfg2 = load_cfg()
            t2 = cfg2["red_drum"]["ranges"][0] if target == "red_drum" else cfg2[target]
            cv2.setTrackbarPos("H low",  win, t2.get("h_low", 0))
            cv2.setTrackbarPos("H high", win, t2.get("h_high", 180))
            cv2.setTrackbarPos("S low",  win, t2.get("s_low", 0))
            cv2.setTrackbarPos("S high", win, t2.get("s_high", 255))
            cv2.setTrackbarPos("V low",  win, t2.get("v_low", 0))
            cv2.setTrackbarPos("V high", win, t2.get("v_high", 255))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="blue_drum", choices=TARGETS)
    tune(ap.parse_args().target)
