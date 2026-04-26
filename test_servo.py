import time
import pigpio

SERVO_GPIO_1 = 12
SERVO_GPIO_2 = 13
PULSE_DOWN_US = 1000
PULSE_UP_US   = 1800

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running. Start with: sudo pigpiod")
    raise SystemExit(1)

try:
    print("Lowering lift (1000 us)...")
    pi.set_servo_pulsewidth(SERVO_GPIO_1, PULSE_DOWN_US)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, PULSE_DOWN_US)
    time.sleep(1)

    print("Raising lift (1800 us)...")
    pi.set_servo_pulsewidth(SERVO_GPIO_1, PULSE_UP_US)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, PULSE_UP_US)
    time.sleep(1)

    print("Lowering lift (1000 us)...")
    pi.set_servo_pulsewidth(SERVO_GPIO_1, PULSE_DOWN_US)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, PULSE_DOWN_US)
    time.sleep(1)

finally:
    pi.set_servo_pulsewidth(SERVO_GPIO_1, 0)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, 0)
    pi.stop()
    print("Done.")
