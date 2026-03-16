"""
vision/detectors.py
All object detection in one file.
Each detector is a pure function: takes a BGR frame + config dict,
returns a result dataclass (never None — always has an 'detected' bool).
"""

from dataclasses import dataclass, field
import cv2
import numpy as np


FRAME_W = 640
FRAME_H = 480
FRAME_CX = FRAME_W // 2
FRAME_CY = FRAME_H // 2


# ──────────────────────────────────────────────────────────────────────────────
# Result types
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class FlareResult:
    detected: bool = False
    cx: int = 0          # centroid x
    cy: int = 0          # centroid y
    area: float = 0.0
    side: str = "none"   # "left" or "right" of frame center

@dataclass
class GateResult:
    detected: bool = False
    center_x: int = 0    # estimated gate center x
    left_x: int = 0
    right_x: int = 0
    error_x: int = 0     # gate_center - frame_center (for IBVS yaw)

@dataclass
class GreenMatResult:
    detected: bool = False
    cx: int = 0
    cy: int = 0
    area: float = 0.0

@dataclass
class DrumResult:
    detected: bool = False
    cx: int = 0
    cy: int = 0
    area: float = 0.0
    error_x: int = 0     # drum_cx - frame_cx (for IBVS alignment)
    error_y: int = 0

@dataclass
class DrumsResult:
    red_drums: list = field(default_factory=list)   # list of DrumResult
    blue_drum: DrumResult = field(default_factory=DrumResult)


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _largest_contour(mask: np.ndarray, min_area: float):
    """Return (contour, area, cx, cy) for largest contour above min_area, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return c, area, cx, cy


# ──────────────────────────────────────────────────────────────────────────────
# Orange Flare Detector
# ──────────────────────────────────────────────────────────────────────────────

def detect_flare(frame: np.ndarray, cfg: dict) -> FlareResult:
    """Detect orange flare obstacle via HSV thresholding."""
    c = cfg["orange_flare"]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([c["h_low"], c["s_low"], c["v_low"]])
    upper = np.array([c["h_high"], c["s_high"], c["v_high"]])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    result = _largest_contour(mask, c["min_area"])
    if result is None:
        return FlareResult()
    _, area, cx, cy = result
    return FlareResult(
        detected=True, cx=cx, cy=cy, area=area,
        side="left" if cx < FRAME_CX else "right"
    )


# ──────────────────────────────────────────────────────────────────────────────
# Gate Detector (Canny + Hough lines → two vertical pipes)
# ──────────────────────────────────────────────────────────────────────────────

def detect_gate(frame: np.ndarray, cfg: dict) -> GateResult:
    """Detect rectangular gate frame using Canny edge + Hough lines."""
    c = cfg["gate"]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, c["canny_low"], c["canny_high"])

    lines = cv2.HoughLinesP(
        edges,
        rho=1, theta=np.pi / 180,
        threshold=c["hough_threshold"],
        minLineLength=c["hough_min_line_length"],
        maxLineGap=c["hough_max_line_gap"]
    )
    if lines is None:
        return GateResult()

    tol = c["vertical_angle_tolerance_deg"]
    vertical_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = abs(np.degrees(np.arctan2(abs(x2 - x1), abs(y2 - y1))))
        if angle < tol:
            cx_line = (x1 + x2) // 2
            vertical_lines.append(cx_line)

    if len(vertical_lines) < 2:
        return GateResult()

    vertical_lines.sort()
    # Find two lines with largest separation — those are the gate posts
    best_pair = (vertical_lines[0], vertical_lines[-1])
    separation = best_pair[1] - best_pair[0]
    if separation < c["min_pipe_separation_px"]:
        return GateResult()

    gate_cx = (best_pair[0] + best_pair[1]) // 2
    return GateResult(
        detected=True,
        center_x=gate_cx,
        left_x=best_pair[0],
        right_x=best_pair[1],
        error_x=gate_cx - FRAME_CX
    )


# ──────────────────────────────────────────────────────────────────────────────
# Green Mat Detector
# ──────────────────────────────────────────────────────────────────────────────

def detect_green_mat(frame: np.ndarray, cfg: dict) -> GreenMatResult:
    c = cfg["green_mat"]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([c["h_low"], c["s_low"], c["v_low"]])
    upper = np.array([c["h_high"], c["s_high"], c["v_high"]])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((9, 9), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))

    result = _largest_contour(mask, c["min_area"])
    if result is None:
        return GreenMatResult()
    _, area, cx, cy = result
    return GreenMatResult(detected=True, cx=cx, cy=cy, area=area)


# ──────────────────────────────────────────────────────────────────────────────
# Drum Detector (red + blue)
# ──────────────────────────────────────────────────────────────────────────────

def detect_drums(frame: np.ndarray, cfg: dict) -> DrumsResult:
    """Detect all red drums and the blue drum."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    result = DrumsResult()

    # --- Red drums (wrap around hue 0/180 — combine two ranges) ---
    red_cfg = cfg["red_drum"]
    red_masks = []
    for r in red_cfg["ranges"]:
        lower = np.array([r["h_low"], r["s_low"], r["v_low"]])
        upper = np.array([r["h_high"], r["s_high"], r["v_high"]])
        red_masks.append(cv2.inRange(hsv, lower, upper))
    red_mask = red_masks[0] | red_masks[1]
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < red_cfg["min_area"]:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        result.red_drums.append(DrumResult(
            detected=True, cx=cx, cy=cy, area=area,
            error_x=cx - FRAME_CX, error_y=cy - FRAME_CY
        ))

    # Sort red drums left to right for consistent mapping
    result.red_drums.sort(key=lambda d: d.cx)

    # --- Blue drum ---
    blue_cfg = cfg["blue_drum"]
    lower = np.array([blue_cfg["h_low"], blue_cfg["s_low"], blue_cfg["v_low"]])
    upper = np.array([blue_cfg["h_high"], blue_cfg["s_high"], blue_cfg["v_high"]])
    blue_mask = cv2.inRange(hsv, lower, upper)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    blue_r = _largest_contour(blue_mask, blue_cfg["min_area"])
    if blue_r is not None:
        _, area, cx, cy = blue_r
        result.blue_drum = DrumResult(
            detected=True, cx=cx, cy=cy, area=area,
            error_x=cx - FRAME_CX, error_y=cy - FRAME_CY
        )

    return result


# ──────────────────────────────────────────────────────────────────────────────
# Debug overlay (optional — for tuning runs)
# ──────────────────────────────────────────────────────────────────────────────

def draw_debug(frame: np.ndarray, flare: FlareResult, gate: GateResult,
               drums: DrumsResult) -> np.ndarray:
    out = frame.copy()
    # Crosshair
    cv2.line(out, (FRAME_CX, 0), (FRAME_CX, FRAME_H), (255, 255, 255), 1)
    cv2.line(out, (0, FRAME_CY), (FRAME_W, FRAME_CY), (255, 255, 255), 1)

    if flare.detected:
        cv2.circle(out, (flare.cx, flare.cy), 20, (0, 120, 255), 3)
        cv2.putText(out, f"FLARE {flare.side}", (flare.cx + 5, flare.cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 255), 1)

    if gate.detected:
        cv2.line(out, (gate.left_x, 0), (gate.left_x, FRAME_H), (255, 255, 0), 2)
        cv2.line(out, (gate.right_x, 0), (gate.right_x, FRAME_H), (255, 255, 0), 2)
        cv2.circle(out, (gate.center_x, FRAME_CY), 8, (255, 255, 0), -1)
        cv2.putText(out, f"GATE err={gate.error_x}", (gate.center_x, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    for d in drums.red_drums:
        cv2.circle(out, (d.cx, d.cy), 15, (0, 0, 200), 3)
    if drums.blue_drum.detected:
        bd = drums.blue_drum
        cv2.circle(out, (bd.cx, bd.cy), 15, (200, 0, 0), 3)
        cv2.putText(out, f"BLUE err={bd.error_x}", (bd.cx + 5, bd.cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 100, 0), 1)

    return out
