import time
import pigpio

PULSE_DOWN_US = 1000
PULSE_UP_US   = 1800

CANDIDATE_PINS = [12, 13, 18, 19, 21, 22, 23, 24, 25, 26, 27]

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running. Start with: sudo pigpiod")
    raise SystemExit(1)

try:
    for pin in CANDIDATE_PINS:
        print(f"Testing GPIO {pin} — watch for servo movement...")
        pi.set_servo_pulsewidth(pin, PULSE_DOWN_US)
        time.sleep(1)
        pi.set_servo_pulsewidth(pin, PULSE_UP_US)
        time.sleep(1)
        pi.set_servo_pulsewidth(pin, PULSE_DOWN_US)
        time.sleep(1)
        pi.set_servo_pulsewidth(pin, 0)
        print(f"  GPIO {pin} done")
finally:
    for pin in CANDIDATE_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()
    print("Scan complete.")
