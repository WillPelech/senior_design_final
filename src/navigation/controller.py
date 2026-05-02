# =============================================================================
# navigation/controller.py  –  State machine
# =============================================================================
# States:
#   IDLE       – waiting for mission command (key 1 or 2)
#   NAVIGATE   – seek EXIT (purple) to pick up car
#   AT_SPOT    – lift car then run timed L-route to parking spot
#   DELIVER    – seek parking marker (ps1=blue / ps2=red) and drop off
#   AT_EXIT    – lower car, reverse out, seek HOME
#   RETURN     – seek HOME (green)
#   DONE       – mission complete
#   ESTOP      – emergency stop
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
    TEST     = auto()
    DONE     = auto()
    ESTOP    = auto()


class Mission(Enum):
    NONE     = auto()
    CAR1     = auto()   # button 1 — picks up at exit (purple), delivers to blue (ps1)
    CAR2     = auto()   # button 2 — picks up at exit (purple), delivers to red  (ps2)
    RETRIEVE = auto()   # button 3 — picks up at blue (ps1), delivers to exit (purple)


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
        self._shape_lost_frames = 0
        log.info("NavigationController ready – IDLE, awaiting mission")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        if config.LIFT_ENABLED:
            self._motors.lift_down()
        log.info("Robot ready. Press 1 (CAR1→blue), 2 (CAR2→red), or SPACE to stop.")

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
        if key == '1' and self._state == State.IDLE:
            log.info("Mission CAR1 started (→ blue/PS1)")
            self._mission = Mission.CAR1
            self._steer_pid.reset()
            self._transition(State.NAVIGATE)
        elif key == '2' and self._state == State.IDLE:
            log.info("Mission CAR2 started (→ red/PS2)")
            self._mission = Mission.CAR2
            self._steer_pid.reset()
            self._transition(State.NAVIGATE)
        elif key == '3' and self._state == State.IDLE:
            log.info("Mission RETRIEVE started (blue/PS1 → purple/EXIT → green/HOME)")
            self._mission = Mission.RETRIEVE
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
            log.info("TEST mode")
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

        # Safety timeout on long-running navigation states
        if self._state in (State.NAVIGATE, State.DELIVER, State.RETURN):
            if self._state_elapsed() > config.SAFETY_NAV_TIMEOUT_S:
                log.error("Navigation timeout – ESTOP")
                self._motors.brake()
                self._transition(State.ESTOP)

        if config.DEBUG_PRINT_TELEMETRY and self._tick_count % config.DEBUG_TELEMETRY_EVERY_N == 0:
            log.info("[TELEM] state=%-10s mission=%-5s ps1=%s ps2=%s home=%s exit=%s",
                     self._state.name, self._mission.name,
                     det.ps1_found, det.ps2_found, det.home_found, det.exit_found)

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _do_navigate(self, det: DetectionResult) -> None:
        """Seek pick-up marker: blue (PS1) for RETRIEVE, purple (EXIT) for CAR1/CAR2."""
        target = 'ps1' if self._mission == Mission.RETRIEVE else 'exit'
        label  = 'blue/PS1' if self._mission == Mission.RETRIEVE else 'purple/EXIT'
        if self._seek_shape(det, target):
            log.info("Arrived at %s — lifting car", label)
            self._motors.stop()
            self._transition(State.AT_SPOT)

    def _do_at_spot(self, det: DetectionResult) -> None:
        """Lift car then run a timed L-route to face the drop-off marker."""
        self._motors.stop()
        if config.LIFT_ENABLED:
            log.info("Lifting car...")
            self._motors.lift_up()

        # RETRIEVE spins the opposite direction to CAR1/CAR2
        spin = config.MOTOR_SEARCH_SPIN_SPEED
        if self._mission == Mission.RETRIEVE:
            turn1 = (-spin,  spin)   # left wheel backward first
            turn2 = ( spin, -spin)   # right wheel backward second
        else:
            turn1 = ( spin, -spin)   # right wheel backward first
            turn2 = (-spin,  spin)   # left wheel backward second

        log.info("L-route: backing out...")
        self._motors.backward(config.LIFT_BACKUP_SPEED)
        time.sleep(config.LIFT_BACKUP_TIME_S)
        self._motors.stop()

        log.info("L-route: first turn...")
        self._motors.set_motors(*turn1)
        time.sleep(config.LIFT_TURN_TIME_S)
        self._motors.stop()

        log.info("L-route: driving forward along wall...")
        self._motors.forward(config.LIFT_BACKUP_SPEED)
        time.sleep(config.DELIVER_FORWARD_TIME_S)
        self._motors.stop()

        log.info("L-route: second turn to face drop-off...")
        self._motors.set_motors(*turn2)
        time.sleep(config.DELIVER_TURN_TIME_S)
        self._motors.stop()

        self._steer_pid.reset()
        self._transition(State.DELIVER)

    def _do_deliver(self, det: DetectionResult) -> None:
        """Seek drop-off marker: purple (EXIT) for RETRIEVE, blue/red for CAR1/CAR2."""
        if self._mission == Mission.RETRIEVE:
            target = 'exit'
            speed  = config.MOTOR_CARRY_SPEED
            close_area = config.SHAPE_CLOSE_AREA
        elif self._mission == Mission.CAR2:
            target = 'ps2'
            speed  = config.MOTOR_CARRY_SPEED_PS2
            close_area = config.SHAPE_CLOSE_AREA_PS2
        else:
            target = 'ps1'
            speed  = config.MOTOR_CARRY_SPEED
            close_area = config.SHAPE_CLOSE_AREA_SPOT
        if self._seek_shape(det, target, speed=speed, close_area=close_area):
            log.info("Arrived at %s — dropping off car", target.upper())
            self._motors.stop()
            self._transition(State.AT_EXIT)

    def _do_at_exit(self, det: DetectionResult) -> None:
        """Lower car, reverse out, then seek HOME."""
        self._motors.stop()
        if config.LIFT_ENABLED:
            log.info("Lowering car...")
            self._motors.lift_down()
        log.info("Backing out from parking spot...")
        self._motors.backward(config.DROP_OFF_BACKUP_SPEED)
        time.sleep(config.DROP_OFF_BACKUP_TIME_S)
        self._motors.stop()
        log.info("Car clear. Seeking HOME.")
        self._steer_pid.reset()
        self._transition(State.RETURN)

    def _do_return(self, det: DetectionResult) -> None:
        """Seek final home marker: red (PS2) for CAR1, green (HOME) for CAR2/RETRIEVE."""
        target = 'ps2' if self._mission == Mission.CAR1 else 'home'
        label  = 'red/PS2' if self._mission == Mission.CAR1 else 'green/HOME'
        if self._seek_shape(det, target):
            log.info("%s reached. Mission complete.", label)
            self._motors.stop()
            self._mission = Mission.NONE
            self._transition(State.DONE)

    def _do_test(self, det: DetectionResult) -> None:
        """Seek EXIT, lift, back out — quick hardware test."""
        if self._seek_shape(det, 'exit'):
            log.info("TEST: reached EXIT — lifting and backing out")
            self._motors.stop()
            if config.LIFT_ENABLED:
                self._motors.lift_up()
                self._motors.backward(config.LIFT_BACKUP_SPEED)
                time.sleep(config.LIFT_BACKUP_TIME_S)
                self._motors.stop()
            self._transition(State.DONE)

    # ------------------------------------------------------------------
    # Shape seeking
    # ------------------------------------------------------------------

    def _seek_shape(self, det: DetectionResult, shape: str,
                    speed: float = None, close_area: int = None) -> bool:
        """
        PID-steer toward shape. Spin in place when lost.
        Returns True when area >= close_area (close enough to stop).
        """
        spd     = speed      if speed      is not None else config.MOTOR_BASE_SPEED
        stop_at = close_area if close_area is not None else config.SHAPE_CLOSE_AREA
        found = getattr(det, f'{shape}_found')
        x_err = getattr(det, f'{shape}_x_error')
        area  = getattr(det, f'{shape}_area')

        if not found:
            self._shape_lost_frames += 1
            if self._shape_lost_frames > 6:
                self._motors.set_motors(config.MOTOR_SEARCH_SPIN_SPEED,
                                        -config.MOTOR_SEARCH_SPIN_SPEED)
            else:
                self._motors.forward(spd)
            return False

        self._shape_lost_frames = 0

        if area >= stop_at:
            return True

        if abs(x_err) < config.STEER_DEAD_ZONE_PX:
            self._steer_pid.reset()
            self._motors.forward(spd)
        else:
            correction = self._steer_pid.compute(x_err)
            left  = self._clamp(spd + correction, lo=config.MOTOR_MIN_SPEED)
            right = self._clamp(spd - correction, lo=config.MOTOR_MIN_SPEED)
            self._motors.set_motors(left, right)
        return False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: State) -> None:
        log.info("State: %s → %s", self._state.name, new_state.name)
        self._state = new_state
        self._state_entry_time = time.monotonic()
        self._shape_lost_frames = 0

    def _state_elapsed(self) -> float:
        return time.monotonic() - self._state_entry_time

    def reload_pid_gains(self) -> None:
        self._steer_pid.kp = config.STEER_KP
        self._steer_pid.ki = config.STEER_KI
        self._steer_pid.kd = config.STEER_KD

    @staticmethod
    def _clamp(v: float, lo: float = -config.MOTOR_MAX_SPEED) -> float:
        return max(lo, min(config.MOTOR_MAX_SPEED, v))
