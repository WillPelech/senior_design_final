# =============================================================================
# camera/capture.py  –  Threaded Pi Camera stream via picamera2
# =============================================================================
# Runs the camera in a background thread so the main control loop always has
# the latest frame available without blocking on I/O.
#
# All camera parameters (resolution, FPS) come from config.py.
# =============================================================================

import logging
import threading
import time
from typing import Optional

import numpy as np

# picamera2 is only available on a real Pi; we guard the import so the module
# can still be imported on a dev machine (frames will just be None).
try:
    from picamera2 import Picamera2
    from libcamera import controls as libcamera_controls
    _PICAMERA_AVAILABLE = True
except ImportError:
    _PICAMERA_AVAILABLE = False

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src import config

log = logging.getLogger(__name__)


class CameraStream:
    """
    Continuously captures frames from the Pi Camera in a daemon thread.

    Usage:
        stream = CameraStream()
        stream.start()
        frame = stream.read()   # numpy BGR array, or None if not ready
        stream.stop()
    """

    def __init__(self) -> None:
        self._frame: Optional[np.ndarray] = None
        self._frame_time: float = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._camera: Optional[object] = None
        self._frame_count: int = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> "CameraStream":
        """Start the background capture thread. Returns self for chaining."""
        if not _PICAMERA_AVAILABLE:
            log.warning(
                "picamera2 not available – running in STUB mode. "
                "Frames will be solid-grey placeholders."
            )
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        log.info(
            "Camera stream started (%dx%d @ %d FPS)",
            config.CAMERA_WIDTH, config.CAMERA_HEIGHT, config.CAMERA_FPS,
        )
        return self

    def stop(self) -> None:
        """Signal the capture thread to stop and wait for it to finish."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._camera is not None and _PICAMERA_AVAILABLE:
            try:
                self._camera.stop()
            except Exception:
                pass
        log.info("Camera stream stopped. Total frames captured: %d", self._frame_count)

    def read(self) -> Optional[np.ndarray]:
        """
        Return the most recent frame as a BGR numpy array (H x W x 3).
        Returns None if no frame has been captured yet.
        """
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def read_timestamp(self) -> float:
        """Return the time.monotonic() timestamp of the most recent frame."""
        with self._lock:
            return self._frame_time

    def is_alive(self) -> bool:
        """True if the capture thread is still running."""
        return self._thread is not None and self._thread.is_alive()

    def age_seconds(self) -> float:
        """Seconds since the last frame was captured. Used for timeout checks."""
        with self._lock:
            if self._frame_time == 0.0:
                return float("inf")
            return time.monotonic() - self._frame_time

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _capture_loop(self) -> None:
        if _PICAMERA_AVAILABLE:
            self._capture_loop_real()
        elif _CV2_AVAILABLE:
            self._capture_loop_usb()
        else:
            self._capture_loop_stub()

    def _capture_loop_real(self) -> None:
        """Capture loop using picamera2 on real hardware."""
        cam = Picamera2()
        self._camera = cam

        # Configure for video capture at the resolution set in config
        video_config = cam.create_video_configuration(
            main={
                "size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT),
                "format": "BGR888",  # OpenCV-native format, no conversion needed
            },
            controls={
                "FrameRate": float(config.CAMERA_FPS),
                # Auto-exposure and auto-white-balance on by default
            },
        )
        cam.configure(video_config)
        cam.start()

        # Allow auto-exposure to settle before the main loop uses frames
        time.sleep(0.5)

        log.debug("picamera2 started successfully")

        while self._running:
            try:
                frame = cam.capture_array("main")  # BGR numpy array
                with self._lock:
                    self._frame = frame
                    self._frame_time = time.monotonic()
                    self._frame_count += 1
            except Exception as exc:
                log.error("Camera capture error: %s", exc)
                time.sleep(0.1)

        cam.stop()

    def _capture_loop_usb(self) -> None:
        """Capture loop using a USB webcam via OpenCV."""
        cap = cv2.VideoCapture(config.USB_CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

        if not cap.isOpened():
            log.error("USB camera (index %d) failed to open – falling back to stub", config.USB_CAMERA_INDEX)
            self._capture_loop_stub()
            return

        log.info("USB camera opened at index %d", config.USB_CAMERA_INDEX)

        while self._running:
            ret, frame = cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                    self._frame_time = time.monotonic()
                    self._frame_count += 1
            else:
                log.warning("USB camera read failed")
                time.sleep(0.1)

        cap.release()

    def _capture_loop_stub(self) -> None:
        """
        Stub loop for development on non-Pi machines.
        Produces a solid-grey frame at the configured resolution and FPS.
        """
        interval = 1.0 / config.CAMERA_FPS
        log.debug("Stub capture loop running at %d FPS", config.CAMERA_FPS)
        while self._running:
            stub_frame = np.full(
                (config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3),
                fill_value=100,
                dtype=np.uint8,
            )
            with self._lock:
                self._frame = stub_frame
                self._frame_time = time.monotonic()
                self._frame_count += 1
            time.sleep(interval)
