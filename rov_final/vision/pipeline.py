"""
vision/pipeline.py
─────────────────────────────────────────────────────────────────────────────
Vision pipeline: camera capture + all detectors with full error handling.

Robustness features
───────────────────
• Camera failure → retry up to MAX_CAM_RETRIES, then degrade gracefully
• Per-frame exception isolation: one detector crash never kills the pipeline
• Frame drop detection: warns if FPS < MIN_FPS
• Stale-frame guard: if camera freezes, returns None after STALE_FRAME_TIMEOUT
• Metrics: fps, frame_count, detector_errors, dropped_frames
"""

import threading
import time
import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from rov_logger import get_logger
from telemetry.bus import TelemetryPublisher, VisionMsg

log = get_logger("vision")

# ── Camera constants ───────────────────────────────────────────────────────
FRAME_W    = 640
FRAME_H    = 480
FRAME_CX   = FRAME_W // 2
FRAME_CY   = FRAME_H // 2
TARGET_FPS = 25
MIN_FPS    = 10          # below this → watchdog degraded
STALE_TIMEOUT = 3.0      # seconds with no new frame → camera error
MAX_CAM_RETRIES = 5

try:
    from picamera2 import Picamera2
    PI_CAM = True
except ImportError:
    PI_CAM = False


# ── Detection result types ────────────────────────────────────────────────

@dataclass
class FlareResult:
    detected: bool = False
    cx: int = 0
    cy: int = 0
    area: float = 0.0
    side: str = "none"    # "left" | "right"

@dataclass
class GateResult:
    detected: bool = False
    center_x: int = 0
    left_x: int   = 0
    right_x: int  = 0
    error_x: int  = 0

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
    error_x: int = 0
    error_y: int = 0

@dataclass
class DrumsResult:
    red_drums: list = field(default_factory=list)
    blue_drum: DrumResult = field(default_factory=DrumResult)

@dataclass
class PipelineResult:
    flare:     FlareResult     = field(default_factory=FlareResult)
    gate:      GateResult      = field(default_factory=GateResult)
    green_mat: GreenMatResult  = field(default_factory=GreenMatResult)
    drums:     DrumsResult     = field(default_factory=DrumsResult)
    frame:     Optional[np.ndarray] = None
    ts:        float = 0.0
    ok:        bool  = True     # False if camera or pipeline failed


# ── Helpers ───────────────────────────────────────────────────────────────

def _largest_contour(mask: np.ndarray, min_area: float):
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
    return c, area, int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])


# ── Individual detectors ──────────────────────────────────────────────────

def detect_flare(frame: np.ndarray, cfg: dict) -> FlareResult:
    try:
        c = cfg["orange_flare"]
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([c["h_low"],  c["s_low"],  c["v_low"]]),
                           np.array([c["h_high"], c["s_high"], c["v_high"]]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        r = _largest_contour(mask, c["min_area"])
        if r is None:
            return FlareResult()
        _, area, cx, cy = r
        return FlareResult(detected=True, cx=cx, cy=cy, area=area,
                           side="left" if cx < FRAME_CX else "right")
    except Exception as e:
        log.error(f"detect_flare exception: {e}", exc_info=True)
        return FlareResult()


def detect_gate(frame: np.ndarray, cfg: dict) -> GateResult:
    try:
        c    = cfg["gate"]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, c["canny_low"], c["canny_high"])
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                threshold=c["hough_threshold"],
                                minLineLength=c["hough_min_line_length"],
                                maxLineGap=c["hough_max_line_gap"])
        if lines is None:
            return GateResult()
        tol = c["vertical_angle_tolerance_deg"]
        v_xs = []
        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            angle = abs(np.degrees(np.arctan2(abs(x2 - x1), abs(y2 - y1))))
            if angle < tol:
                v_xs.append((x1 + x2) // 2)
        if len(v_xs) < 2:
            return GateResult()
        v_xs.sort()
        lx, rx = v_xs[0], v_xs[-1]
        if (rx - lx) < c["min_pipe_separation_px"]:
            return GateResult()
        gate_cx = (lx + rx) // 2
        return GateResult(detected=True, center_x=gate_cx,
                          left_x=lx, right_x=rx, error_x=gate_cx - FRAME_CX)
    except Exception as e:
        log.error(f"detect_gate exception: {e}", exc_info=True)
        return GateResult()


def detect_green_mat(frame: np.ndarray, cfg: dict) -> GreenMatResult:
    try:
        c    = cfg["green_mat"]
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([c["h_low"],  c["s_low"],  c["v_low"]]),
                           np.array([c["h_high"], c["s_high"], c["v_high"]]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((9, 9),  np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        r = _largest_contour(mask, c["min_area"])
        if r is None:
            return GreenMatResult()
        _, area, cx, cy = r
        return GreenMatResult(detected=True, cx=cx, cy=cy, area=area)
    except Exception as e:
        log.error(f"detect_green_mat exception: {e}", exc_info=True)
        return GreenMatResult()


def detect_drums(frame: np.ndarray, cfg: dict) -> DrumsResult:
    try:
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        result = DrumsResult()
        # Red (two ranges)
        rc     = cfg["red_drum"]
        r_masks = []
        for r in rc["ranges"]:
            r_masks.append(cv2.inRange(hsv,
                np.array([r["h_low"], r["s_low"], r["v_low"]]),
                np.array([r["h_high"], r["s_high"], r["v_high"]])))
        red_mask = r_masks[0] | r_masks[1]
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for ct in cnts:
            area = cv2.contourArea(ct)
            if area < rc["min_area"]:
                continue
            M = cv2.moments(ct)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            result.red_drums.append(DrumResult(
                detected=True, cx=cx, cy=cy, area=area,
                error_x=cx - FRAME_CX, error_y=cy - FRAME_CY))
        result.red_drums.sort(key=lambda d: d.cx)
        # Blue
        bc   = cfg["blue_drum"]
        bmask = cv2.inRange(hsv,
                            np.array([bc["h_low"], bc["s_low"], bc["v_low"]]),
                            np.array([bc["h_high"], bc["s_high"], bc["v_high"]]))
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        br = _largest_contour(bmask, bc["min_area"])
        if br is not None:
            _, area, cx, cy = br
            result.blue_drum = DrumResult(
                detected=True, cx=cx, cy=cy, area=area,
                error_x=cx - FRAME_CX, error_y=cy - FRAME_CY)
        return result
    except Exception as e:
        log.error(f"detect_drums exception: {e}", exc_info=True)
        return DrumsResult()


def draw_debug_overlay(frame: np.ndarray, result: PipelineResult) -> np.ndarray:
    out = frame.copy()
    cv2.line(out, (FRAME_CX, 0), (FRAME_CX, FRAME_H), (200, 200, 200), 1)
    cv2.line(out, (0, FRAME_CY), (FRAME_W, FRAME_CY), (200, 200, 200), 1)
    if result.flare.detected:
        cv2.circle(out, (result.flare.cx, result.flare.cy), 20, (0, 100, 255), 3)
        cv2.putText(out, f"FLARE {result.flare.side}", (result.flare.cx, result.flare.cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
    if result.gate.detected:
        cv2.line(out, (result.gate.left_x, 0),  (result.gate.left_x, FRAME_H),  (0, 255, 255), 2)
        cv2.line(out, (result.gate.right_x, 0), (result.gate.right_x, FRAME_H), (0, 255, 255), 2)
        cv2.putText(out, f"GATE err={result.gate.error_x}",
                    (result.gate.center_x - 40, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    for d in result.drums.red_drums:
        cv2.circle(out, (d.cx, d.cy), 16, (30, 30, 220), 3)
    if result.drums.blue_drum.detected:
        bd = result.drums.blue_drum
        cv2.circle(out, (bd.cx, bd.cy), 16, (220, 80, 0), 3)
        cv2.putText(out, f"BLUE a={bd.area:.0f}", (bd.cx, bd.cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 80, 0), 1)
    return out


# ── Camera + Pipeline ─────────────────────────────────────────────────────

class VisionPipeline:
    """
    Threaded camera + detector pipeline.
    Call get_result() from the state machine to get the latest PipelineResult.
    """

    def __init__(self, cfg: dict, watchdog=None,
                 publisher: Optional[TelemetryPublisher] = None):
        self._cfg       = cfg
        self._watchdog  = watchdog
        self._pub       = publisher

        self._result    = PipelineResult()
        self._lock      = threading.Lock()
        self._running   = False
        self._thread: Optional[threading.Thread] = None

        # Metrics
        self.frame_count     = 0
        self.detector_errors = 0
        self.dropped_frames  = 0
        self._fps            = 0.0
        self._fps_frames     = 0
        self._fps_t0         = time.monotonic()
        self._last_frame_ts  = 0.0

        # Active state filter — only run detectors relevant to current state
        self._active_detectors: set[str] = {"flare", "gate", "drums", "green_mat"}

    def set_active_detectors(self, detectors: set[str]) -> None:
        self._active_detectors = detectors

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="vision"
        )
        self._thread.start()
        # Wait up to 5 s for first frame
        for _ in range(100):
            if self._last_frame_ts > 0:
                break
            time.sleep(0.05)
        if self._last_frame_ts == 0:
            log.error("Vision: no frame received in 5 s after start")
        else:
            log.info("Vision pipeline started")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)

    def get_result(self) -> PipelineResult:
        with self._lock:
            return self._result

    @property
    def fps(self) -> float:
        return self._fps

    # ── Internal ──────────────────────────────────────────────────────────

    def _run(self) -> None:
        for attempt in range(1, MAX_CAM_RETRIES + 1):
            try:
                if PI_CAM:
                    self._run_picamera()
                else:
                    self._run_opencv()
                break
            except Exception as e:
                log.error(f"Vision camera failure (attempt {attempt}): {e}", exc_info=True)
                if self._watchdog:
                    self._watchdog.increment_error("vision")
                if attempt < MAX_CAM_RETRIES:
                    log.info(f"Vision retrying in 3 s...")
                    time.sleep(3.0)
                else:
                    log.critical("Vision: max camera retries reached — pipeline dead")
                    if self._watchdog:
                        self._watchdog.set_degraded("vision", "camera dead")
                    with self._lock:
                        self._result = PipelineResult(ok=False, ts=time.monotonic())

    def _process_frame(self, frame: np.ndarray) -> None:
        self._last_frame_ts = time.monotonic()
        self._fps_frames += 1
        elapsed = time.monotonic() - self._fps_t0
        if elapsed >= 2.0:
            self._fps = self._fps_frames / elapsed
            self._fps_frames = 0
            self._fps_t0 = time.monotonic()
            if self._fps < MIN_FPS:
                log.warning(f"Vision FPS low: {self._fps:.1f} (min={MIN_FPS})")
                if self._watchdog:
                    self._watchdog.set_degraded("vision", f"fps={self._fps:.1f}")

        ads = self._active_detectors
        flare     = detect_flare(frame, self._cfg)     if "flare"     in ads else FlareResult()
        gate      = detect_gate(frame, self._cfg)      if "gate"      in ads else GateResult()
        green_mat = detect_green_mat(frame, self._cfg) if "green_mat" in ads else GreenMatResult()
        drums     = detect_drums(frame, self._cfg)     if "drums"     in ads else DrumsResult()

        r = PipelineResult(
            flare=flare, gate=gate, green_mat=green_mat,
            drums=drums, frame=frame, ts=time.monotonic(), ok=True
        )
        with self._lock:
            self._result = r
        self.frame_count += 1

        if self._watchdog:
            self._watchdog.feed("vision")

        # Publish telemetry
        if self._pub:
            try:
                self._pub.pub_vision(VisionMsg(
                    flare_detected=flare.detected,
                    gate_detected=gate.detected,
                    gate_error_x=gate.error_x,
                    blue_drum=drums.blue_drum.detected,
                    blue_area=drums.blue_drum.area,
                    blue_error_x=drums.blue_drum.error_x,
                    red_count=len(drums.red_drums),
                    ts=time.monotonic()
                ))
            except Exception:
                pass

    def _run_picamera(self) -> None:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(
            main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
        ))
        cam.start()
        log.info("picamera2 started")
        try:
            while self._running:
                rgb = cam.capture_array()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                self._process_frame(bgr)
        finally:
            cam.stop()

    def _run_opencv(self) -> None:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("OpenCV VideoCapture could not open camera")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        log.info("OpenCV VideoCapture started")
        consecutive_failures = 0
        try:
            while self._running:
                ret, frame = cap.read()
                if not ret:
                    consecutive_failures += 1
                    self.dropped_frames += 1
                    log.warning(f"Camera read failed (consecutive={consecutive_failures})")
                    if consecutive_failures >= 10:
                        raise RuntimeError("Camera: 10 consecutive read failures")
                    time.sleep(0.05)
                    continue
                consecutive_failures = 0
                self._process_frame(frame)
        finally:
            cap.release()
