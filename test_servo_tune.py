"""
Servo tuning — control left and right servos independently.

Controls (type command then press Enter):
  w / W   Left servo  UP   +10 / +50 us
  s / S   Left servo  DOWN -10 / -50 us
  e / E   Right servo UP   +10 / +50 us
  d / D   Right servo DOWN -10 / -50 us
  t       Test cycle: both DOWN → UP → DOWN
  q       Quit and print final config values

Both servos start at lowest position (700 us).
Position is autosaved on every move.

Run:
  sudo pigpiod
  source .venv/bin/activate
  python test_servo_tune.py
"""

import time
import pigpio

SERVO_GPIO_LEFT  = 18   # GPIO 18 / Pin 12
SERVO_GPIO_RIGHT = 27   # GPIO 27 / Pin 13

START_US = 700
MAX_US   = 2500
MIN_US   = 500

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running — start with: sudo pigpiod")
    raise SystemExit(1)

left_us  = START_US
right_us = START_US


def set_left(us):
    global left_us
    left_us = max(MIN_US, min(MAX_US, us))
    pi.set_servo_pulsewidth(SERVO_GPIO_LEFT, left_us)
    _status()


def set_right(us):
    global right_us
    right_us = max(MIN_US, min(MAX_US, us))
    pi.set_servo_pulsewidth(SERVO_GPIO_RIGHT, right_us)
    _status()


def _status():
    print(f"  LEFT={left_us:4d} us    RIGHT={right_us:4d} us")


def release():
    pi.set_servo_pulsewidth(SERVO_GPIO_LEFT,  0)
    pi.set_servo_pulsewidth(SERVO_GPIO_RIGHT, 0)


print(__doc__)
print("─" * 42)
print("Starting at lowest position (700 us)...")
set_left(START_US)
set_right(START_US)
print()

try:
    while True:
        cmd = input("> ").strip()
        if   cmd == 'q':  break
        elif cmd == 'w':  set_left(left_us + 10)
        elif cmd == 's':  set_left(left_us - 10)
        elif cmd == 'W':  set_left(left_us + 50)
        elif cmd == 'S':  set_left(left_us - 50)
        elif cmd == 'e':  set_right(right_us + 10)
        elif cmd == 'd':  set_right(right_us - 10)
        elif cmd == 'E':  set_right(right_us + 50)
        elif cmd == 'D':  set_right(right_us - 50)
        elif cmd == 't':
            print("  Test: both DOWN...")
            set_left(START_US); set_right(START_US); time.sleep(1.5)
            print("  Test: both UP...")
            set_left(left_us);  set_right(right_us); time.sleep(1.5)
            print("  Test: both DOWN...")
            set_left(START_US); set_right(START_US)
        else:
            print("  w/s=left  e/d=right  W/S/E/D=x5  t=test  q=quit")
finally:
    release()
    pi.stop()

print(f"\nFinal config — copy to src/config.py:")
print(f"  SERVO_PULSE_DOWN_US = {START_US}")
print(f"  SERVO_PULSE_UP_US   = {max(left_us, right_us)}")
print(f"  (left={left_us} us   right={right_us} us)")
