# =============================================================================
# motors/driver.py  –  Motor HAT + gripper servo abstraction
# =============================================================================
# Provides a clean interface to:
#   - Two drive motors (left / right) via Adafruit Motor HAT (I2C)
#   - One gripper servo (Thingiverse thing:2415) via RPi GPIO PWM or Motor HAT
#
# All pin numbers, speed limits, and servo pulse widths come from config.py.
#
# On non-Pi machines the module runs in STUB mode and prints commands instead
# of sending them to hardware.
# =============================================================================

import logging
import time

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src import config

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Optional hardware imports – guarded for dev-machine compatibility
# ---------------------------------------------------------------------------
try:
    from adafruit_motorkit import MotorKit
    from adafruit_motor import motor as adafruit_motor
    import board
    _HAT_AVAILABLE = True
except ImportError:
    _HAT_AVAILABLE = False

try:
    import pigpio
    _PIGPIO_AVAILABLE = True
except ImportError:
    _PIGPIO_AVAILABLE = False


def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


class MotorDriver:
    """
    Unified driver for the two drive motors and the forklift servo.

    Motor speed convention (matches Adafruit MotorKit):
        +1.0  = full forward
         0.0  = stopped
        -1.0  = full reverse

    Gripper positions (Thingiverse thing:2415 parallel-jaw gripper):
        OPEN   – jaws wide, robot free to drive toward car
        CLOSED – jaws grip the car body sides
    """

    def __init__(self) -> None:
        self._kit = None
        self._pi  = None
        self._gripper_closed = False

        self._init_motors()
        self._init_servo()

        log.info(
            "MotorDriver ready | HAT=%s | pigpio=%s | servo_pins=%d,%d",
            _HAT_AVAILABLE, _PIGPIO_AVAILABLE,
            config.SERVO_GPIO_PIN, config.SERVO_GPIO_PIN_2,
        )

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _init_motors(self) -> None:
        if config.STUB_MOTORS:
            log.warning("STUB_MOTORS=True – motors disabled for camera testing")
            return
        if _HAT_AVAILABLE:
            try:
                self._kit = MotorKit(i2c=board.I2C())
                log.info("Adafruit Motor HAT initialised via I2C")
            except Exception as exc:
                log.error("Motor HAT init failed: %s – falling back to STUB", exc)
                self._kit = None
        else:
            log.warning("adafruit_motorkit not installed – motors in STUB mode")

    def _init_servo(self) -> None:
        if config.STUB_MOTORS:
            return
        if not _PIGPIO_AVAILABLE:
            log.warning("pigpio not available – servo in STUB mode")
            return
        try:
            self._pi = pigpio.pi()
            if not self._pi.connected:
                log.error("pigpio daemon not running (sudo pigpiod) – servo in STUB mode")
                self._pi = None
                return
            self._pi.set_servo_pulsewidth(config.SERVO_GPIO_PIN,   config.SERVO_PULSE_DOWN_US)
            self._pi.set_servo_pulsewidth(config.SERVO_GPIO_PIN_2, config.SERVO_PULSE_DOWN_US)
            log.info("Lift servos ready on GPIO %d and %d", config.SERVO_GPIO_PIN, config.SERVO_GPIO_PIN_2)
        except Exception as exc:
            log.error("pigpio init failed: %s – servo in STUB mode", exc)
            self._pi = None

    # ------------------------------------------------------------------
    # Drive motors
    # ------------------------------------------------------------------

    def set_motors(self, left: float, right: float) -> None:
        """
        Set both drive motor speeds simultaneously.

        Args:
            left:  Speed for left motor  (-1.0 reverse … +1.0 forward)
            right: Speed for right motor (-1.0 reverse … +1.0 forward)
        """
        left  = _clamp(self._apply_deadband(left))
        right = _clamp(self._apply_deadband(right))

        if _HAT_AVAILABLE and self._kit is not None:
            # Motor HAT: M2 = left drive, M1 = right drive
            self._kit.motor2.throttle = left
            self._kit.motor1.throttle = right
        else:
            log.debug("STUB motors | L=%.2f  R=%.2f", left, right)

    def stop(self) -> None:
        """Immediately stop both drive motors (coast, not brake)."""
        self.set_motors(0.0, 0.0)

    def brake(self) -> None:
        """Hard brake both motors (active braking via MotorKit BRAKE mode)."""
        if _HAT_AVAILABLE and self._kit is not None:
            self._kit.motor1.throttle = None  # None = RELEASE / float
            self._kit.motor2.throttle = None
        else:
            log.debug("STUB brake")

    def forward(self, speed: float = None) -> None:
        """Drive both motors forward at the same speed."""
        spd = speed if speed is not None else config.MOTOR_BASE_SPEED
        self.set_motors(spd, spd)

    def backward(self, speed: float = None) -> None:
        """Drive both motors in reverse at the same speed."""
        spd = speed if speed is not None else config.MOTOR_BASE_SPEED
        self.set_motors(-spd, -spd)

    def spin_left(self, speed: float = None) -> None:
        """Spin in place counter-clockwise (left motor back, right motor forward)."""
        spd = speed if speed is not None else config.MOTOR_SEARCH_SPIN_SPEED
        self.set_motors(-spd, spd)

    def spin_right(self, speed: float = None) -> None:
        """Spin in place clockwise (left motor forward, right motor back)."""
        spd = speed if speed is not None else config.MOTOR_SEARCH_SPIN_SPEED
        self.set_motors(spd, -spd)

    # ------------------------------------------------------------------
    # Lift servo
    # ------------------------------------------------------------------

    def lift_up(self) -> None:
        """Raise the lift platform to carry the car."""
        self._set_servo_pulse(config.SERVO_PULSE_UP_US)
        self._gripper_closed = True
        log.info("Lift UP (%.0fus)", config.SERVO_PULSE_UP_US)

    def lift_down(self) -> None:
        """Lower the lift platform to travel/release position."""
        self._set_servo_pulse(config.SERVO_PULSE_DOWN_US)
        self._gripper_closed = False
        log.info("Lift DOWN (%.0fus)", config.SERVO_PULSE_DOWN_US)

    # kept for compatibility
    def gripper_open(self) -> None:
        self.lift_down()

    def gripper_close(self) -> None:
        self.lift_up()

    @property
    def gripper_is_closed(self) -> bool:
        return self._gripper_closed

    def _set_servo_pulse(self, pulse_us: float) -> None:
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(config.SERVO_GPIO_PIN,   int(pulse_us))
            self._pi.set_servo_pulsewidth(config.SERVO_GPIO_PIN_2, int(pulse_us))
            log.debug("Lift servos pulse=%dus", int(pulse_us))
        else:
            log.debug("STUB servo pulse=%dus", int(pulse_us))

    # ------------------------------------------------------------------
    # Dead-band
    # ------------------------------------------------------------------

    def _apply_deadband(self, speed: float) -> float:
        """
        Zero out very small speed commands to avoid motor buzz.
        Uses MOTOR_MIN_SPEED from config as the dead-band threshold.
        """
        if abs(speed) < config.MOTOR_MIN_SPEED:
            return 0.0
        return speed

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def cleanup(self) -> None:
        """Release motors and servo. Call on shutdown."""
        self.stop()
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(config.SERVO_GPIO_PIN,   0)
            self._pi.set_servo_pulsewidth(config.SERVO_GPIO_PIN_2, 0)
            self._pi.stop()
        log.info("MotorDriver cleaned up")
