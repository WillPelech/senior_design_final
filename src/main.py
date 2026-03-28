#!/usr/bin/env python3
# =============================================================================
# main.py  –  AI Valet Robot  –  Entry Point
# =============================================================================
# Wires together:
#   CameraStream   → continuous Pi Camera frames
#   ArucoDetector  → marker detection + pose estimation per frame
#   MotorDriver    → Motor HAT + forklift servo
#   NavigationController → PID state machine
#
# Run on the Pi with:
#   python3 src/main.py
#
# Keyboard shortcuts (when DEBUG_SHOW_PREVIEW = True):
#   q / ESC  – quit cleanly
#   r        – reload PID gains from config (live tuning)
#   s        – skip to next state (debug override)
#   SPACE    – emergency stop then quit
# =============================================================================

import logging
import os
import signal
import sys
import time

# ---------------------------------------------------------------------------
# Set up logging before importing anything else
# ---------------------------------------------------------------------------
import src.config as config   # noqa: E402  (config first so LOG_LEVEL is available)

logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL, logging.INFO),
    format="%(asctime)s  %(levelname)-8s  %(name)s – %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")

# ---------------------------------------------------------------------------
# Local imports
# ---------------------------------------------------------------------------
from src.camera.capture import CameraStream
from src.detection.aruco_detector import ArucoDetector
from src.motors.driver import MotorDriver
from src.navigation.controller import NavigationController, State

# cv2 for the optional preview window
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Frame-save helper
# ---------------------------------------------------------------------------
def _maybe_save_frame(frame, tick: int) -> None:
    if not config.DEBUG_SAVE_FRAMES:
        return
    if tick % config.DEBUG_SAVE_EVERY_N != 0:
        return
    if not _CV2_AVAILABLE:
        return
    os.makedirs(config.DEBUG_SAVE_DIR, exist_ok=True)
    path = os.path.join(config.DEBUG_SAVE_DIR, f"frame_{tick:06d}.jpg")
    cv2.imwrite(path, frame)


# ---------------------------------------------------------------------------
# Main run loop
# ---------------------------------------------------------------------------
def run() -> None:
    log.info("=" * 60)
    log.info("AI Valet Robot  –  starting up")
    log.info("=" * 60)

    # Instantiate subsystems
    camera   = CameraStream()
    detector = ArucoDetector()
    motors   = MotorDriver()
    nav      = NavigationController(motors)

    # Graceful shutdown on SIGINT / SIGTERM
    _shutdown_requested = [False]

    def _signal_handler(sig, _frame):
        log.warning("Signal %d received – requesting shutdown", sig)
        _shutdown_requested[0] = True

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Start camera stream
    camera.start()

    # Allow camera to warm up
    log.info("Waiting for first camera frame…")
    deadline = time.monotonic() + config.SAFETY_CAMERA_TIMEOUT_S
    while camera.read() is None:
        if time.monotonic() > deadline:
            log.error("Camera did not produce a frame in time – aborting")
            camera.stop()
            motors.cleanup()
            sys.exit(1)
        time.sleep(0.05)
    log.info("Camera ready")

    # Lower forks and enter SEARCHING
    nav.start()

    tick = 0
    loop_interval = config.PID_DT   # seconds per main-loop iteration

    try:
        while not _shutdown_requested[0]:
            loop_start = time.monotonic()

            # ── Camera health check ──────────────────────────────────────
            if camera.age_seconds() > config.SAFETY_CAMERA_TIMEOUT_S:
                log.error("Camera timeout – ESTOP")
                motors.brake()
                break

            # ── Grab frame ──────────────────────────────────────────────
            frame = camera.read()
            if frame is None:
                time.sleep(0.01)
                continue

            # ── Detect ──────────────────────────────────────────────────
            det = detector.detect(frame)

            # ── Navigate ────────────────────────────────────────────────
            nav.tick(det)

            # ── Preview window ──────────────────────────────────────────
            if config.DEBUG_SHOW_PREVIEW and _CV2_AVAILABLE:
                display = det.annotated if det.annotated is not None else frame
                _overlay_state(display, nav.state, det, tick)
                cv2.imshow("AI Valet", display)
                key = cv2.waitKey(1)
                key = key & 0xFF if key is not None else -1

                if key in (ord('q'), 27):       # q or ESC
                    log.info("Quit key pressed")
                    _shutdown_requested[0] = True

                elif key == ord('r'):            # reload PID gains
                    log.info("Reloading PID gains from config…")
                    import importlib
                    importlib.reload(config)
                    nav.reload_pid_gains()

                elif key == ord(' '):            # emergency stop
                    log.warning("SPACE pressed – emergency stop")
                    motors.brake()
                    _shutdown_requested[0] = True

            # ── Save frame ──────────────────────────────────────────────
            if det.annotated is not None:
                _maybe_save_frame(det.annotated, tick)

            # ── Mission complete? ────────────────────────────────────────
            if nav.state in (State.DONE, State.ESTOP):
                log.info("Mission ended in state %s", nav.state.name)
                break

            # ── Rate-limit main loop ─────────────────────────────────────
            elapsed = time.monotonic() - loop_start
            sleep_for = loop_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

            tick += 1

    finally:
        log.info("Shutting down…")
        nav.shutdown()
        camera.stop()
        motors.cleanup()
        if _CV2_AVAILABLE and config.DEBUG_SHOW_PREVIEW:
            cv2.destroyAllWindows()
        log.info("Shutdown complete. Total ticks: %d", tick)


# ---------------------------------------------------------------------------
# Preview overlay helper
# ---------------------------------------------------------------------------
def _overlay_state(
    img,
    state: State,
    det,
    tick: int,
) -> None:
    """Draw HUD info onto the preview frame in-place."""
    if not _CV2_AVAILABLE:
        return

    # State name (top-left)
    cv2.putText(
        img,
        f"STATE: {state.name}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    # Tick counter
    cv2.putText(
        img,
        f"tick {tick}",
        (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )

    if det.found_car:
        # Distance bar
        bar_max_m  = 1.0                           # full bar = 1 metre
        bar_width  = int((det.distance / bar_max_m) * 200)
        bar_width  = min(bar_width, 200)
        cv2.rectangle(img, (10, 60), (10 + bar_width, 75), (0, 200, 100), -1)
        cv2.rectangle(img, (10, 60), (210, 75), (255, 255, 255), 1)
        cv2.putText(
            img, f"dist {det.distance:.2f}m",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 100), 1, cv2.LINE_AA,
        )

        # X-error indicator (centre line)
        cx = img.shape[1] // 2
        cy = img.shape[0] // 2
        # 1 pixel per 0.5 mm
        offset_px = int(det.x_error * 2000)
        cv2.arrowedLine(
            img,
            (cx, cy),
            (cx + offset_px, cy),
            (0, 80, 255), 2, tipLength=0.3,
        )

    # Key hints at bottom
    cv2.putText(
        img,
        "q=quit  r=reload-PID  SPACE=estop",
        (10, img.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (150, 150, 150),
        1,
        cv2.LINE_AA,
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    run()
