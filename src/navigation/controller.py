# =============================================================================
# navigation/controller.py  –  PID control + robot state machine
# =============================================================================
# Implements the full autonomous behaviour loop:
#
#   SEARCHING  → ALIGNING → APPROACHING → GRIPPING → RETURNING → DONE
#
# Each call to tick(detection_result) advances the state machine by one step
# and issues motor commands via MotorDriver.
#
# All thresholds, PID gains, speed limits, and timeouts live in config.py.
# =============================================================================

import logging
import math
import time
from enum import Enum, auto
from typing import Optional

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src import config
from src.detection.aruco_detector import DetectionResult
from src.motors.driver import MotorDriver

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Robot states
# ---------------------------------------------------------------------------
class State(Enum):
    SEARCHING   = auto()  # Spinning, scanning for car marker
    ALIGNING    = auto()  # Marker found, centring it in frame (lateral PID)
    APPROACHING = auto()  # Centred, driving forward (depth PID)
    GRIPPING    = auto()  # Stopped at dock distance, closing gripper jaws
    RETURNING   = auto()  # Gripper closed, navigating back to home marker
    DONE        = auto()  # Mission complete, motors off
    ESTOP       = auto()  # Emergency stop (safety trigger)


# ---------------------------------------------------------------------------
# PID controller (generic, used for both steering and approach)
# ---------------------------------------------------------------------------
class PID:
    """
    Simple discrete PID with anti-windup integral clamping.

    All gains and limits are passed in at construction so each PID instance
    is independent and testable.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_max: float,
        dt: float,
        name: str = "PID",
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        self.dt = dt
        self.name = name

        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self) -> None:
        """Reset integrator and derivative state (call on state transitions)."""
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float) -> float:
        """
        Compute PID output for the given error.

        Args:
            error: current error signal (setpoint - measured)

        Returns:
            Control output (sign and magnitude depend on gains)
        """
        # Proportional
        p = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * self.dt
        self._integral = max(-self.i_max, min(self.i_max, self._integral))
        i = self.ki * self._integral

        # Derivative (backward difference)
        derivative = (error - self._prev_error) / self.dt
        d = self.kd * derivative
        self._prev_error = error

        output = p + i + d

        if config.DEBUG_PRINT_TELEMETRY:
            log.debug(
                "[%s] err=%.4f  P=%.4f  I=%.4f  D=%.4f  out=%.4f",
                self.name, error, p, i, d, output,
            )

        return output

    def update_gains(self, kp: float, ki: float, kd: float) -> None:
        """Hot-swap PID gains without resetting the integrator."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        log.info("[%s] gains updated kp=%.3f ki=%.3f kd=%.3f", self.name, kp, ki, kd)


# ---------------------------------------------------------------------------
# Main navigation controller
# ---------------------------------------------------------------------------
class NavigationController:
    """
    Runs the robot state machine each loop tick.

    Usage:
        ctrl = NavigationController(motor_driver)
        ctrl.start()
        while running:
            det = detector.detect(frame)
            ctrl.tick(det)
        ctrl.shutdown()
    """

    def __init__(self, motors: MotorDriver) -> None:
        self._motors = motors
        self._state = State.SEARCHING
        self._state_entry_time = time.monotonic()

        # PID controllers – gains pulled from config at construction
        self._steer_pid = PID(
            kp=config.STEER_KP,
            ki=config.STEER_KI,
            kd=config.STEER_KD,
            i_max=config.STEER_I_MAX,
            dt=config.PID_DT,
            name="Steer",
        )
        self._approach_pid = PID(
            kp=config.APPROACH_KP,
            ki=config.APPROACH_KI,
            kd=config.APPROACH_KD,
            i_max=config.APPROACH_I_MAX,
            dt=config.PID_DT,
            name="Approach",
        )

        # Search bookkeeping
        self._search_cycle = 0
        self._lost_frame_count = 0

        # Telemetry counter
        self._tick_count = 0

        log.info("NavigationController initialised in state %s", self._state.name)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Enter SEARCHING state with gripper open."""
        self._motors.gripper_open()
        self._transition(State.SEARCHING)

    def shutdown(self) -> None:
        """Emergency-safe shutdown: stop motors, open gripper to release car."""
        self._motors.stop()
        self._motors.gripper_open()
        log.info("NavigationController shutdown")

    @property
    def state(self) -> State:
        return self._state

    def tick(self, det: DetectionResult) -> None:
        """
        Advance the state machine by one step using the latest detection result.

        Call this once per camera frame from the main loop.
        """
        self._tick_count += 1

        # ── Safety checks (run regardless of state) ──────────────────────
        if self._check_safety(det):
            return   # ESTOP triggered

        # ── State dispatch ────────────────────────────────────────────────
        if self._state == State.SEARCHING:
            self._do_searching(det)
        elif self._state == State.ALIGNING:
            self._do_aligning(det)
        elif self._state == State.APPROACHING:
            self._do_approaching(det)
        elif self._state == State.GRIPPING:
            self._do_gripping(det)
        elif self._state == State.RETURNING:
            self._do_returning(det)
        elif self._state in (State.DONE, State.ESTOP):
            self._motors.stop()

        # ── Periodic telemetry ────────────────────────────────────────────
        if (
            config.DEBUG_PRINT_TELEMETRY
            and self._tick_count % config.DEBUG_TELEMETRY_EVERY_N == 0
        ):
            self._print_telemetry(det)

    def reload_pid_gains(self) -> None:
        """
        Re-read PID gains from config without restarting.
        Useful for live-tuning: edit config.py values at runtime and call this.
        """
        self._steer_pid.update_gains(config.STEER_KP, config.STEER_KI, config.STEER_KD)
        self._approach_pid.update_gains(
            config.APPROACH_KP, config.APPROACH_KI, config.APPROACH_KD
        )

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _do_searching(self, det: DetectionResult) -> None:
        """Spin slowly and check each step for the car marker."""
        if det.found_car:
            log.info("Marker found! Transitioning to ALIGNING")
            self._lost_frame_count = 0
            self._search_cycle = 0
            self._steer_pid.reset()
            self._approach_pid.reset()
            self._motors.stop()
            self._transition(State.ALIGNING)
            return

        # Check search timeout
        elapsed = self._state_elapsed()
        if elapsed > config.SAFETY_SEARCH_TIMEOUT_S:
            log.warning("Search timeout (%.0fs). Stopping.", elapsed)
            self._transition(State.DONE)
            self._motors.stop()
            return

        # Spin step
        self._motors.spin_right(config.MOTOR_SEARCH_SPIN_SPEED)
        time.sleep(config.SEARCH_STEP_PAUSE_S)
        self._motors.stop()

    def _do_aligning(self, det: DetectionResult) -> None:
        """Centre the car marker horizontally using the steering PID."""
        if not det.found_car:
            self._lost_frame_count += 1
            if self._lost_frame_count >= config.ARUCO_LOST_FRAME_THRESHOLD:
                log.warning("Marker lost in ALIGNING – back to SEARCHING")
                self._motors.stop()
                self._transition(State.SEARCHING)
            return
        self._lost_frame_count = 0

        x_error = det.x_error   # meters; positive = car is to the right

        if abs(x_error) < config.ALIGN_X_THRESHOLD_M:
            log.info("Aligned! x_error=%.4fm → APPROACHING", x_error)
            self._steer_pid.reset()
            self._motors.stop()
            self._transition(State.APPROACHING)
            return

        # Steer in place: turn toward the marker
        correction = self._steer_pid.compute(x_error)
        # Positive correction → car is RIGHT → spin right (left fwd, right back)
        left_speed  =  config.MOTOR_SEARCH_SPIN_SPEED + correction
        right_speed = -(config.MOTOR_SEARCH_SPIN_SPEED + correction)
        left_speed  = self._clamp_speed(left_speed)
        right_speed = self._clamp_speed(right_speed)
        self._motors.set_motors(left_speed, right_speed)

    def _do_approaching(self, det: DetectionResult) -> None:
        """Drive forward toward the marker while keeping it centred."""
        # Timeout guard
        if self._state_elapsed() > config.SAFETY_APPROACH_TIMEOUT_S:
            log.warning("Approach timeout – stopping")
            self._motors.stop()
            self._transition(State.ESTOP)
            return

        if not det.found_car:
            self._lost_frame_count += 1
            if self._lost_frame_count >= config.ARUCO_LOST_FRAME_THRESHOLD:
                log.warning("Marker lost in APPROACHING – backing up, re-searching")
                self._motors.backward(config.SEARCH_BACKUP_SPEED)
                time.sleep(config.SEARCH_BACKUP_TIME_S)
                self._motors.stop()
                self._transition(State.SEARCHING)
            return
        self._lost_frame_count = 0

        distance = det.distance    # meters to marker
        x_error  = det.x_error    # meters lateral offset

        # ── Reached dock distance? ────────────────────────────────────────
        # Add GRIPPER_STANDOFF_EXTRA_M so jaws have room to close around the car
        dock_dist = config.DOCK_DISTANCE_M + config.GRIPPER_STANDOFF_EXTRA_M
        if distance <= dock_dist:
            log.info("Docked! distance=%.3fm → GRIPPING", distance)
            self._motors.stop()
            time.sleep(config.MOTOR_SETTLE_TIME_S)
            self._transition(State.GRIPPING)
            return

        # ── Creep speed when close ────────────────────────────────────────
        base_speed = (
            config.MOTOR_CREEP_SPEED
            if distance < config.FAR_DISTANCE_M * 0.4
            else config.MOTOR_BASE_SPEED
        )

        # ── Approach PID: compute forward speed component ─────────────────
        # error = how far we still need to go (positive = need to move forward)
        depth_error  = distance - config.DOCK_DISTANCE_M
        forward_cmd  = self._approach_pid.compute(depth_error)
        forward_cmd  = max(0.0, min(forward_cmd, config.MOTOR_MAX_SPEED))

        # ── Steering PID: compute differential ───────────────────────────
        steer_cmd = self._steer_pid.compute(x_error)

        # Combine: differential drive mix
        left_speed  = forward_cmd - steer_cmd
        right_speed = forward_cmd + steer_cmd
        left_speed  = self._clamp_speed(left_speed)
        right_speed = self._clamp_speed(right_speed)

        self._motors.set_motors(left_speed, right_speed)

    def _do_gripping(self, det: DetectionResult) -> None:
        """
        Close the gripper jaws around the car body.
        This state executes once then immediately transitions.
        The robot must be stationary before this is called.
        """
        self._motors.stop()
        log.info("Closing gripper jaws...")
        self._motors.gripper_close()   # blocks for SERVO_TRAVEL_TIME_S
        log.info("Gripper closed. Car secured.")

        if config.ARUCO_HOME_MARKER_ID is not None:
            self._steer_pid.reset()
            self._approach_pid.reset()
            self._transition(State.RETURNING)
        else:
            log.info("No home marker configured – mission DONE")
            self._transition(State.DONE)

    def _do_returning(self, det: DetectionResult) -> None:
        """Navigate to the home marker using the same steer + approach PIDs."""
        if not det.found_home:
            # Spin search for home marker
            self._motors.spin_right(config.MOTOR_SEARCH_SPIN_SPEED)
            return

        assert det.home_tvec is not None  # guaranteed by found_home guard above
        distance = float(det.home_tvec[2])
        x_error  = float(det.home_tvec[0])

        if distance <= config.HOME_DISTANCE_M:
            log.info("Home reached! Releasing gripper.")
            self._motors.stop()
            if config.GRIPPER_AUTO_RELEASE_ON_RETURN:
                self._motors.gripper_open()
            self._transition(State.DONE)
            return

        steer_cmd   = self._steer_pid.compute(x_error)
        depth_error = distance - config.HOME_DISTANCE_M
        forward_cmd = self._approach_pid.compute(depth_error)
        forward_cmd = max(0.0, min(forward_cmd, config.MOTOR_RETURN_SPEED))

        left_speed  = self._clamp_speed(forward_cmd - steer_cmd)
        right_speed = self._clamp_speed(forward_cmd + steer_cmd)
        self._motors.set_motors(left_speed, right_speed)

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def _check_safety(self, det: DetectionResult) -> bool:
        """
        Run all safety checks. Returns True if ESTOP was triggered.
        The ESTOP check for minimum distance only fires while approaching
        (not while gripping/returning/done).
        """
        if self._state == State.APPROACHING and det.found_car:
            if det.distance < config.SAFETY_MIN_DISTANCE_M:
                log.error(
                    "SAFETY: distance %.3fm < min %.3fm – ESTOP",
                    det.distance, config.SAFETY_MIN_DISTANCE_M,
                )
                self._motors.brake()
                self._transition(State.ESTOP)
                return True
        return False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: State) -> None:
        log.info("State: %s → %s", self._state.name, new_state.name)
        self._state = new_state
        self._state_entry_time = time.monotonic()
        self._lost_frame_count = 0

    def _state_elapsed(self) -> float:
        """Seconds since entering the current state."""
        return time.monotonic() - self._state_entry_time

    @staticmethod
    def _clamp_speed(speed: float) -> float:
        return max(-config.MOTOR_MAX_SPEED, min(config.MOTOR_MAX_SPEED, speed))

    def _print_telemetry(self, det: DetectionResult) -> None:
        log.info(
            "[TELEM] state=%-12s | found=%s | dist=%.3fm | x_err=%+.3fm | "
            "ids=%s | tick=%d",
            self._state.name,
            det.found_car,
            det.distance,
            det.x_error,
            det.all_ids,
            self._tick_count,
        )
