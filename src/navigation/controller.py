# =============================================================================
# navigation/controller.py  –  Line-following state machine
# =============================================================================
# States:
#   IDLE       – waiting for mission command (key 1 or 2)
#   NAVIGATE   – line following toward target spot
#   AT_SPOT    – stopped at parking spot, lifting car
#   DELIVER    – line following toward EXIT
#   AT_EXIT    – stopped at exit, lowering car
#   RETURN     – line following back to HOME
#   DONE       – mission complete, motors off
#   ESTOP      – emergency stop
#
# Missions:
#   Mission.CAR1 – pick up from PS1 (blue square), deliver to EXIT
#   Mission.CAR2 – pick up from PS2 (red square), deliver to EXIT
# =============================================================================

import logging
import time
from enum import Enum, auto

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src import config
from src.detection.aruco_detector import DetectionResult
from src.motors.driver import MotorDriver

log = logging.getLogger(__name__)


class State(Enum):
    IDLE     = auto()
    NAVIGATE = auto()
    AT_SPOT  = auto()
    DELIVER  = auto()
    AT_EXIT  = auto()
    RETURN   = auto()
    DONE     = auto()
    ESTOP    = auto()


class Mission(Enum):
    NONE = auto()
    CAR1 = auto()   # pick up from PS1 (blue square)
    CAR2 = auto()   # pick up from PS2 (red square)


class PID:
    def __init__(self, kp, ki, kd, i_max, dt, name="PID"):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.i_max = i_max; self.dt = dt; self.name = name
        self._integral = 0.0; self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0; self._prev_error = 0.0

    def compute(self, error):
        p = self.kp * error
        self._integral = max(-self.i_max, min(self.i_max, self._integral + error * self.dt))
        i = self.ki * self._integral
        d = self.kd * (error - self._prev_error) / self.dt
        self._prev_error = error
        return p + i + d


class NavigationController:

    def __init__(self, motors: MotorDriver) -> None:
        self._motors = motors
        self._state  = State.IDLE
        self._mission = Mission.NONE
        self._state_entry_time = time.monotonic()
        self._tick_count = 0
        self._fork_passed = False

        self._steer_pid = PID(
            kp=config.STEER_KP, ki=config.STEER_KI, kd=config.STEER_KD,
            i_max=config.STEER_I_MAX, dt=config.PID_DT, name="Steer"
        )
        log.info("NavigationController ready – IDLE, awaiting mission")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._motors.lift_down()
        log.info("Robot ready. Press 1 (CAR1), 2 (CAR2), or SPACE to stop.")

    def shutdown(self) -> None:
        self._motors.stop()
        self._motors.lift_down()

    @property
    def state(self) -> State:
        return self._state

    @property
    def mission(self) -> Mission:
        return self._mission

    def command(self, key: str) -> None:
        """Handle a key command from the SSH UI."""
        if key == '1' and self._state == State.IDLE:
            log.info("Mission CAR1 started")
            self._mission = Mission.CAR1
            self._fork_passed = False
            self._steer_pid.reset()
            self._transition(State.NAVIGATE)
        elif key == '2' and self._state == State.IDLE:
            log.info("Mission CAR2 started")
            self._mission = Mission.CAR2
            self._fork_passed = False
            self._steer_pid.reset()
            self._transition(State.NAVIGATE)
        elif key == 'h':
            log.info("Manual return HOME")
            self._mission = Mission.NONE
            self._transition(State.RETURN)
        elif key == ' ':
            log.warning("ESTOP commanded")
            self._motors.brake()
            self._transition(State.ESTOP)
        elif key == 'r':
            log.info("Resetting to IDLE")
            self._motors.stop()
            self._mission = Mission.NONE
            self._transition(State.IDLE)

    def tick(self, det: DetectionResult) -> None:
        self._tick_count += 1

        if self._state == State.IDLE:
            self._motors.stop()

        elif self._state == State.NAVIGATE:
            self._do_navigate(det)

        elif self._state == State.AT_SPOT:
            self._do_at_spot(det)

        elif self._state == State.DELIVER:
            self._do_deliver(det)

        elif self._state == State.AT_EXIT:
            self._do_at_exit(det)

        elif self._state == State.RETURN:
            self._do_return(det)

        elif self._state in (State.DONE, State.ESTOP):
            self._motors.stop()

        # Safety timeout
        if self._state in (State.NAVIGATE, State.DELIVER, State.RETURN):
            if self._state_elapsed() > config.SAFETY_NAV_TIMEOUT_S:
                log.error("Navigation timeout – ESTOP")
                self._motors.brake()
                self._transition(State.ESTOP)

        if config.DEBUG_PRINT_TELEMETRY and self._tick_count % config.DEBUG_TELEMETRY_EVERY_N == 0:
            log.info("[TELEM] state=%-10s mission=%-5s line=%s x_err=%+.0f fork=%s ps1=%s ps2=%s home=%s exit=%s",
                     self._state.name, self._mission.name,
                     det.line_found, det.line_x_error,
                     det.fork_detected, det.at_ps1, det.at_ps2, det.at_home, det.at_exit)

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _do_navigate(self, det: DetectionResult) -> None:
        """Follow black line toward target parking spot."""
        # Check if we've reached the fork
        if det.fork_detected and not self._fork_passed:
            log.info("Fork detected")
            self._fork_passed = True
            # CAR1 → blue branch (continue following blue tape)
            # CAR2 → black branch (ignore blue, keep on black)
            # Both cases just continue line following — the tape layout guides the robot

        # Check if we've arrived at the target spot
        if self._mission == Mission.CAR1 and det.at_ps1:
            log.info("Arrived at PS1")
            self._motors.stop()
            self._transition(State.AT_SPOT)
            return
        if self._mission == Mission.CAR2 and det.at_ps2:
            log.info("Arrived at PS2")
            self._motors.stop()
            self._transition(State.AT_SPOT)
            return

        self._follow_line(det, config.MOTOR_BASE_SPEED)

    def _do_at_spot(self, det: DetectionResult) -> None:
        """Lift the car then transition to DELIVER."""
        self._motors.stop()
        log.info("Lifting car...")
        self._motors.lift_up()
        time.sleep(config.SERVO_TRAVEL_TIME_S)
        log.info("Car lifted. Navigating to EXIT.")
        self._fork_passed = False
        self._steer_pid.reset()
        self._transition(State.DELIVER)

    def _do_deliver(self, det: DetectionResult) -> None:
        """Follow line to EXIT."""
        if det.at_exit:
            log.info("Arrived at EXIT")
            self._motors.stop()
            self._transition(State.AT_EXIT)
            return
        self._follow_line(det, config.MOTOR_BASE_SPEED)

    def _do_at_exit(self, det: DetectionResult) -> None:
        """Lower car at exit then return home."""
        self._motors.stop()
        log.info("Lowering car at EXIT...")
        self._motors.lift_down()
        time.sleep(config.SERVO_TRAVEL_TIME_S)
        log.info("Car delivered. Returning home.")
        self._steer_pid.reset()
        self._transition(State.RETURN)

    def _do_return(self, det: DetectionResult) -> None:
        """Follow line back to HOME."""
        if det.at_home:
            log.info("Home reached. Mission complete.")
            self._motors.stop()
            self._mission = Mission.NONE
            self._transition(State.DONE)
            return
        self._follow_line(det, config.MOTOR_BASE_SPEED)

    # ------------------------------------------------------------------
    # Line following
    # ------------------------------------------------------------------

    def _follow_line(self, det: DetectionResult, base_speed: float) -> None:
        if not det.line_found:
            # Line lost – slow creep forward hoping to reacquire
            self._motors.forward(config.MOTOR_CREEP_SPEED)
            return

        correction = self._steer_pid.compute(det.line_x_error)
        left  = self._clamp(base_speed + correction)
        right = self._clamp(base_speed - correction)
        self._motors.set_motors(left, right)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: State) -> None:
        log.info("State: %s → %s", self._state.name, new_state.name)
        self._state = new_state
        self._state_entry_time = time.monotonic()

    def _state_elapsed(self) -> float:
        return time.monotonic() - self._state_entry_time

    def reload_pid_gains(self) -> None:
        self._steer_pid.kp = config.STEER_KP
        self._steer_pid.ki = config.STEER_KI
        self._steer_pid.kd = config.STEER_KD

    @staticmethod
    def _clamp(v: float) -> float:
        return max(-config.MOTOR_MAX_SPEED, min(config.MOTOR_MAX_SPEED, v))
