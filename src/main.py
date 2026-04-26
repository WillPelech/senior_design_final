#!/usr/bin/env python3
# =============================================================================
# main.py  –  AI Valet Robot  –  Entry Point
# =============================================================================
# SSH UI controls (press key then Enter):
#   1      – Pick up Car 1 from PS1, deliver to EXIT, return HOME
#   2      – Pick up Car 2 from PS2, deliver to EXIT, return HOME
#   h      – Return robot to HOME
#   r      – Reset to IDLE
#   SPACE  – Emergency stop
#   q      – Quit
# =============================================================================

import logging
import os
import select
import signal
import sys
import termios
import time
import tty

import src.config as config

logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL, logging.INFO),
    format="%(asctime)s  %(levelname)-8s  %(name)s – %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")

from src.camera.capture import CameraStream
from src.detection.aruco_detector import ArucoDetector
from src.motors.driver import MotorDriver
from src.navigation.controller import NavigationController, State

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Non-blocking single-keypress reader (works over SSH, no display needed)
# ---------------------------------------------------------------------------

class KeyReader:
    """Reads single keypresses from stdin without blocking or requiring Enter."""

    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old_settings = None
        self._enabled = False
        try:
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setraw(self._fd)
            self._enabled = True
        except Exception:
            # Not a real TTY (e.g. piped input) – fall back to no key input
            self._enabled = False

    def read(self) -> str:
        """Return a key if one is waiting, else empty string."""
        if not self._enabled:
            return ""
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                return sys.stdin.read(1)
        except Exception:
            pass
        return ""

    def restore(self):
        if self._enabled and self._old_settings is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# HUD overlay for optional preview window
# ---------------------------------------------------------------------------

def _overlay_hud(img, state: State, det, tick: int, mission_name: str) -> None:
    if not _CV2_AVAILABLE:
        return
    cv2.putText(img, f"STATE: {state.name}  MISSION: {mission_name}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, f"tick {tick}  line={'YES' if det.line_found else 'NO'}  x={det.line_x_error:+.0f}px",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
    flags = []
    if det.fork_detected: flags.append("FORK")
    if det.at_ps1:        flags.append("PS1")
    if det.at_ps2:        flags.append("PS2")
    if det.at_home:       flags.append("HOME")
    if det.at_exit:       flags.append("EXIT")
    if flags:
        cv2.putText(img, " ".join(flags), (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(img, "1=CAR1  2=CAR2  h=HOME  SPC=STOP  q=QUIT",
                (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def _print_ui_banner():
    print("\n" + "=" * 50)
    print("  AI Valet Robot – SSH Control")
    print("=" * 50)
    print("  1       Pick up Car 1 (PS1) → EXIT → HOME")
    print("  2       Pick up Car 2 (PS2) → EXIT → HOME")
    print("  h       Return robot to HOME")
    print("  r       Reset to IDLE")
    print("  SPACE   Emergency stop")
    print("  q       Quit")
    print("=" * 50 + "\n")
    sys.stdout.flush()


def run() -> None:
    log.info("AI Valet Robot starting up")

    camera   = CameraStream()
    detector = ArucoDetector()
    motors   = MotorDriver()
    nav      = NavigationController(motors)
    keys     = KeyReader()

    _shutdown_requested = [False]

    def _signal_handler(sig, _frame):
        log.warning("Signal %d – shutdown requested", sig)
        _shutdown_requested[0] = True

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    camera.start()
    log.info("Waiting for camera...")
    deadline = time.monotonic() + config.SAFETY_CAMERA_TIMEOUT_S
    while camera.read() is None:
        if time.monotonic() > deadline:
            log.error("Camera timeout – aborting")
            camera.stop(); motors.cleanup(); keys.restore()
            sys.exit(1)
        time.sleep(0.05)
    log.info("Camera ready")

    nav.start()
    _print_ui_banner()

    tick = 0
    loop_interval = config.PID_DT

    try:
        while not _shutdown_requested[0]:
            loop_start = time.monotonic()

            # Camera health
            if camera.age_seconds() > config.SAFETY_CAMERA_TIMEOUT_S:
                log.error("Camera timeout – ESTOP")
                motors.brake()
                break

            # Grab frame
            frame = camera.read()
            if frame is None:
                time.sleep(0.01)
                continue

            # Detect
            det = detector.detect(frame)

            # Key input
            key = keys.read()
            if key:
                if key == 'q':
                    log.info("Quit key pressed")
                    _shutdown_requested[0] = True
                else:
                    nav.command(key)
                    # Print current state after command
                    print(f"  → State: {nav.state.name}  Mission: {nav.mission.name}")
                    sys.stdout.flush()

            # Navigate
            nav.tick(det)

            # Print state periodically to terminal
            if tick % 30 == 0:
                print(f"  State: {nav.state.name:<10}  Mission: {nav.mission.name:<5}  "
                      f"Line: {'YES' if det.line_found else 'NO '}"
                      f"  x_err: {det.line_x_error:+.0f}px"
                      f"  {'FORK' if det.fork_detected else ''}"
                      f"{'PS1' if det.at_ps1 else ''}"
                      f"{'PS2' if det.at_ps2 else ''}"
                      f"{'HOME' if det.at_home else ''}"
                      f"{'EXIT' if det.at_exit else ''}")
                sys.stdout.flush()

            # Optional preview window
            if config.DEBUG_SHOW_PREVIEW and _CV2_AVAILABLE:
                display = det.annotated if det.annotated is not None else frame
                _overlay_hud(display, nav.state, det, tick, nav.mission.name)
                cv2.imshow("AI Valet", display)
                cv2.waitKey(1)

            # Mission done
            if nav.state in (State.DONE, State.ESTOP):
                print(f"\n  Mission ended: {nav.state.name}")
                if nav.state == State.DONE:
                    print("  Press 1 or 2 for next mission, q to quit.")
                    nav._transition(State.IDLE)
                sys.stdout.flush()

            # Rate limit
            elapsed = time.monotonic() - loop_start
            sleep_for = loop_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

            tick += 1

    finally:
        log.info("Shutting down...")
        keys.restore()
        nav.shutdown()
        camera.stop()
        motors.cleanup()
        if _CV2_AVAILABLE and config.DEBUG_SHOW_PREVIEW:
            cv2.destroyAllWindows()
        log.info("Shutdown complete. Total ticks: %d", tick)


if __name__ == "__main__":
    run()
