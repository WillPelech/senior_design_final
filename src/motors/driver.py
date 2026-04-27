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
import math
import time
from typing import Tuple

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
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


# ---------------------------------------------------------------------------
# Helper: clamp a value to [-1, 1]
# ---------------------------------------------------------------------------
def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Helper: convert microseconds → duty cycle percentage for RPi.GPIO PWM
# Formula: duty = (pulse_us / period_us) * 100
# ---------------------------------------------------------------------------
def _us_to_duty(pulse_us: float, freq_hz: float) -> float:
    period_us = 1_000_000.0 / freq_hz
    return (pulse_us / period_us) * 100.0


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
        self._pwm  = None   # lift servo 1 (GPIO 18 / Pin 12)
        self._pwm2 = None   # lift servo 2 (GPIO 27 / Pin 13)
        self._gripper_closed = False

        self._init_motors()
        self._init_servo()

        log.info(
            "MotorDriver ready | HAT=%s | GPIO=%s | servo_pin=%d",
            _HAT_AVAILABLE, _GPIO_AVAILABLE, config.SERVO_GPIO_PIN,
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
        if config.SERVO_USE_HAT_CHANNEL:
            log.info("Servo will use Motor HAT channel %d", config.SERVO_HAT_CHANNEL)
            return

        if _GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                # Servo 1 — Pin 12 / GPIO 18
                GPIO.setup(config.SERVO_GPIO_PIN, GPIO.OUT)
                self._pwm = GPIO.PWM(config.SERVO_GPIO_PIN, config.SERVO_PWM_FREQ_HZ)
                self._pwm.start(_us_to_duty(config.SERVO_PULSE_DOWN_US, config.SERVO_PWM_FREQ_HZ))
                # Servo 2 — Pin 13 / GPIO 27
                GPIO.setup(config.SERVO_GPIO_PIN_2, GPIO.OUT)
                self._pwm2 = GPIO.PWM(config.SERVO_GPIO_PIN_2, config.SERVO_PWM_FREQ_HZ)
                self._pwm2.start(_us_to_duty(config.SERVO_PULSE_DOWN_US, config.SERVO_PWM_FREQ_HZ))
                log.info("Lift servos started on GPIO %d and %d", config.SERVO_GPIO_PIN, config.SERVO_GPIO_PIN_2)
            except Exception as exc:
                log.error("Servo GPIO init failed: %s – servo in STUB mode", exc)
                self._pwm = None
                self._pwm2 = None
        else:
            log.warning("RPi.GPIO not available – servo in STUB mode")

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
            # Motor HAT: M1 = left drive, M2 = right drive
            self._kit.motor1.throttle = left
            self._kit.motor2.throttle = right
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
        """Send pulse to both lift servos in sync."""
        if config.SERVO_USE_HAT_CHANNEL:
            self._set_servo_hat(pulse_us)
        elif self._pwm is not None:
            duty = _us_to_duty(pulse_us, config.SERVO_PWM_FREQ_HZ)
            self._pwm.ChangeDutyCycle(duty)
            if self._pwm2 is not None:
                self._pwm2.ChangeDutyCycle(duty)
            log.debug("Lift servos pulse=%.0fus duty=%.2f%%", pulse_us, duty)
        else:
            log.debug("STUB servo pulse=%.0fus", pulse_us)

    def _set_servo_hat(self, pulse_us: float) -> None:
        """Drive servo through the Motor HAT servo channel."""
        if not (_HAT_AVAILABLE and self._kit is not None):
            log.debug("STUB HAT servo pulse=%.0fus", pulse_us)
            return
        # MotorKit servo uses angle 0–180 degrees; convert pulse → angle
        # Pulse range 1000us=0° to 2000us=180° (adjust for your servo)
        pulse_min_us = 1000.0
        pulse_max_us = 2000.0
        angle = (pulse_us - pulse_min_us) / (pulse_max_us - pulse_min_us) * 180.0
        angle = max(0.0, min(180.0, angle))
        try:
            servo = getattr(self._kit, f"servo{config.SERVO_HAT_CHANNEL}")
            servo.angle = angle
        except AttributeError:
            log.error("Invalid SERVO_HAT_CHANNEL=%d", config.SERVO_HAT_CHANNEL)

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
        if self._pwm is not None:
            self._pwm.stop()
        if self._pwm2 is not None:
            self._pwm2.stop()
        if _GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        log.info("MotorDriver cleaned up")
