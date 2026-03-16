"""
vision/camera.py
Threaded camera capture using picamera2.
Provides latest frame to vision modules without blocking.
"""

import threading
import time
import numpy as np

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False
    print("[Camera] picamera2 not found — falling back to OpenCV VideoCapture")

import cv2


class Camera:
    """
    Threaded camera capture.
    Always returns the latest frame; never blocks the caller.

    Usage:
        cam = Camera()
        cam.start()
        frame = cam.get_frame()   # numpy BGR array, or None if not ready
        cam.stop()
    """

    WIDTH  = 640
    HEIGHT = 480
    FPS    = 30

    def __init__(self, use_picamera: bool = True):
        self._use_picamera = use_picamera and PICAMERA2_AVAILABLE
        self._frame = None
        self._lock  = threading.Lock()
        self._running = False
        self._thread  = None
        self._fps_counter = 0
        self._fps_start   = time.monotonic()
        self.actual_fps   = 0.0

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        # Wait for first frame
        for _ in range(50):
            if self._frame is not None:
                break
            time.sleep(0.05)
        print(f"[Camera] started ({'picamera2' if self._use_picamera else 'OpenCV'})")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def get_frame(self) -> np.ndarray | None:
        """Return latest BGR frame, or None if not yet available."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def _update_fps(self) -> None:
        self._fps_counter += 1
        elapsed = time.monotonic() - self._fps_start
        if elapsed >= 2.0:
            self.actual_fps = self._fps_counter / elapsed
            self._fps_counter = 0
            self._fps_start = time.monotonic()

    # ──────────────────────────────────────────────────────────────────
    # Capture loops
    # ──────────────────────────────────────────────────────────────────
    def _capture_loop(self) -> None:
        if self._use_picamera:
            self._picamera_loop()
        else:
            self._opencv_loop()

    def _picamera_loop(self) -> None:
        cam = Picamera2()
        config = cam.create_preview_configuration(
            main={"size": (self.WIDTH, self.HEIGHT), "format": "RGB888"}
        )
        cam.configure(config)
        cam.start()
        try:
            while self._running:
                rgb = cam.capture_array()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                with self._lock:
                    self._frame = bgr
                self._update_fps()
        finally:
            cam.stop()

    def _opencv_loop(self) -> None:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, self.FPS)
        try:
            while self._running:
                ret, frame = cap.read()
                if ret:
                    with self._lock:
                        self._frame = frame
                    self._update_fps()
        finally:
            cap.release()
