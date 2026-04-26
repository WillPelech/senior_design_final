import time
import pigpio

PULSE_DOWN_US = 1000
PULSE_UP_US   = 1800

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running. Start with: sudo pigpiod")
    raise SystemExit(1)

try:
    print("Testing GPIO 12 only...")
    pi.set_servo_pulsewidth(12, PULSE_UP_US)
    time.sleep(2)
    pi.set_servo_pulsewidth(12, PULSE_DOWN_US)
    time.sleep(2)
    pi.set_servo_pulsewidth(12, 0)

    print("Testing GPIO 13 only...")
    pi.set_servo_pulsewidth(13, PULSE_UP_US)
    time.sleep(2)
    pi.set_servo_pulsewidth(13, PULSE_DOWN_US)
    time.sleep(2)
    pi.set_servo_pulsewidth(13, 0)

finally:
    pi.set_servo_pulsewidth(12, 0)
    pi.set_servo_pulsewidth(13, 0)
    pi.stop()
    print("Done.")
