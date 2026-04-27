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
    TEST     = auto()   # follow line until any green shape
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

        self._steer_pid = PID(
            kp=config.STEER_KP, ki=config.STEER_KI, kd=config.STEER_KD,
            i_max=config.STEER_I_MAX, dt=config.PID_DT, name="Steer"
        )
        log.info("NavigationController ready – IDLE, awaiting mission")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        if config.LIFT_ENABLED:
            self._motors.lift_down()
        log.info("Robot ready. Press 1 (CAR1), 2 (CAR2), or SPACE to stop.")

    def shutdown(self) -> None:
        self._motors.stop()
        if config.LIFT_ENABLED:
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

            self._steer_pid.reset()
            self._transition(State.NAVIGATE)
        elif key == '2' and self._state == State.IDLE:
            log.info("Mission CAR2 started")
            self._mission = Mission.CAR2

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
        elif key == 't':
            log.info("TEST: follow line until any green shape")
            self._steer_pid.reset()
            self._transition(State.TEST)
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

        elif self._state == State.TEST:
            self._do_test(det)

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
        """Seek target parking spot shape."""
        target = 'ps1' if self._mission == Mission.CAR1 else 'ps2'
        if self._seek_shape(det, target):
            log.info("Arrived at %s", target.upper())
            self._motors.stop()
            self._transition(State.AT_SPOT)

    def _do_at_spot(self, det: DetectionResult) -> None:
        """Lift the car then seek EXIT."""
        self._motors.stop()
        if config.LIFT_ENABLED:
            log.info("Lifting car...")
            self._motors.lift_up()
            time.sleep(config.SERVO_TRAVEL_TIME_S)
            log.info("Car lifted. Seeking EXIT.")
        self._steer_pid.reset()
        self._transition(State.DELIVER)

    def _do_deliver(self, det: DetectionResult) -> None:
        """Seek EXIT shape."""
        if self._seek_shape(det, 'exit'):
            log.info("Arrived at EXIT")
            self._motors.stop()
            self._transition(State.AT_EXIT)

    def _do_at_exit(self, det: DetectionResult) -> None:
        """Lower car then seek HOME."""
        self._motors.stop()
        if config.LIFT_ENABLED:
            log.info("Lowering car at EXIT...")
            self._motors.lift_down()
            time.sleep(config.SERVO_TRAVEL_TIME_S)
            log.info("Car delivered. Seeking HOME.")
        self._steer_pid.reset()
        self._transition(State.RETURN)

    def _do_test(self, det: DetectionResult) -> None:
        """Seek HOME circle as a simple test."""
        if self._seek_shape(det, 'home'):
            log.info("TEST: reached HOME — stopping")
            self._motors.stop()
            self._transition(State.DONE)

    def _do_return(self, det: DetectionResult) -> None:
        """Seek HOME shape to complete mission."""
        if self._seek_shape(det, 'home'):
            log.info("Home reached. Mission complete.")
            self._motors.stop()
            self._mission = Mission.NONE
            self._transition(State.DONE)

    # ------------------------------------------------------------------
    # Shape seeking (replaces line following)
    # ------------------------------------------------------------------

    def _seek_shape(self, det: DetectionResult, shape: str) -> bool:
        """
        Spin to search for shape, drive toward it when found.
        Returns True when close enough (area >= SHAPE_CLOSE_AREA).
        shape: 'home' | 'ps1' | 'ps2' | 'exit'
        """
        found = getattr(det, f'{shape}_found')
        x_err = getattr(det, f'{shape}_x_error')
        area  = getattr(det, f'{shape}_area')

        if not found:
            # Spin slowly to search
            self._motors.set_motors(config.MOTOR_SEARCH_SPIN_SPEED,
                                    -config.MOTOR_SEARCH_SPIN_SPEED)
            return False

        if area >= config.SHAPE_CLOSE_AREA:
            return True

        # Drive toward shape with steering correction
        correction = self._steer_pid.compute(x_err)
        left  = self._clamp(config.MOTOR_BASE_SPEED + correction, lo=0.15)
        right = self._clamp(config.MOTOR_BASE_SPEED - correction, lo=0.15)
        self._motors.set_motors(left, right)
        return False

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
    def _clamp(v: float, lo: float = -config.MOTOR_MAX_SPEED) -> float:
        return max(lo, min(config.MOTOR_MAX_SPEED, v))
