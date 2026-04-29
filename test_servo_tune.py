"""
Servo tuning — sweeps from 700us to 2300us in 50us steps.
Watch the servos and note which pulse value = fully down and fully up.
Then update src/config.py with those values.

Run:
  sudo pigpiod
  source .venv/bin/activate
  python test_servo_tune.py
"""

import time
import pigpio

SERVO_GPIO_1 = 18
SERVO_GPIO_2 = 27

START_US = 700
END_US   = 2300
STEP_US  = 50
HOLD_S   = 0.8   # seconds at each position

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running. Start with: sudo pigpiod")
    raise SystemExit(1)

try:
    print("Sweeping UP (700 → 2300 us) — watch for fully raised position")
    for us in range(START_US, END_US + 1, STEP_US):
        pi.set_servo_pulsewidth(SERVO_GPIO_1, us)
        pi.set_servo_pulsewidth(SERVO_GPIO_2, us)
        print(f"  {us} us")
        time.sleep(HOLD_S)

    print("\nSweeping DOWN (2300 → 700 us) — watch for fully lowered position")
    for us in range(END_US, START_US - 1, -STEP_US):
        pi.set_servo_pulsewidth(SERVO_GPIO_1, us)
        pi.set_servo_pulsewidth(SERVO_GPIO_2, us)
        print(f"  {us} us")
        time.sleep(HOLD_S)

finally:
    pi.set_servo_pulsewidth(SERVO_GPIO_1, 0)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, 0)
    pi.stop()

print("\nNote the pulse values and update src/config.py:")
print("  SERVO_PULSE_DOWN_US = <value when platform is lowered>")
print("  SERVO_PULSE_UP_US   = <value when platform is raised>")
