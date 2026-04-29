"""
Servo tuning script — find the correct SERVO_PULSE_DOWN_US and SERVO_PULSE_UP_US.

Controls (no Enter needed):
  w / s   raise / lower by 10 us
  W / S   raise / lower by 50 us
  u       send current pulse to UP position (save as up)
  d       send current pulse to DOWN position (save as down)
  t       test full cycle: go down → go up → go down
  q       quit and print final config values

Run:
  sudo pigpiod
  source .venv/bin/activate
  python test_servo_tune.py
"""

import sys
import select
import termios
import tty
import time

try:
    import pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running. Start with: sudo pigpiod")
        sys.exit(1)
    _HW = True
except ImportError:
    print("pigpio not available — running in STUB mode (no servos will move)")
    pi = None
    _HW = False

SERVO_GPIO_1 = 18
SERVO_GPIO_2 = 27

pulse_us   = 1400  # start in middle
pulse_down = 1000
pulse_up   = 1800


def set_pulse(us):
    us = max(500, min(2500, us))
    if _HW:
        pi.set_servo_pulsewidth(SERVO_GPIO_1, us)
        pi.set_servo_pulsewidth(SERVO_GPIO_2, us)
    print(f"  Pulse: {us} us  (down={pulse_down}  up={pulse_up})")
    return us


def release():
    if _HW:
        pi.set_servo_pulsewidth(SERVO_GPIO_1, 0)
        pi.set_servo_pulsewidth(SERVO_GPIO_2, 0)


# Set up raw tty
fd = sys.stdin.fileno()
old_tty = None
try:
    old_tty = termios.tcgetattr(fd)
    tty.setraw(fd)
except Exception:
    pass


def read_key():
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
    except Exception:
        pass
    return ""


print("Servo tuning — move to a position, then press u/d to save as up/down")
print("  w/s = ±10us    W/S = ±50us    u=save up    d=save down    t=test cycle    q=quit\n")

pulse_us = set_pulse(pulse_us)

try:
    while True:
        key = read_key()
        if key == 'q':
            break
        elif key == 'w':
            pulse_us = set_pulse(pulse_us + 10)
        elif key == 's':
            pulse_us = set_pulse(pulse_us - 10)
        elif key == 'W':
            pulse_us = set_pulse(pulse_us + 50)
        elif key == 'S':
            pulse_us = set_pulse(pulse_us - 50)
        elif key == 'u':
            pulse_up = pulse_us
            print(f"  Saved UP = {pulse_up} us")
        elif key == 'd':
            pulse_down = pulse_us
            print(f"  Saved DOWN = {pulse_down} us")
        elif key == 't':
            print("  Test cycle: DOWN → UP → DOWN")
            set_pulse(pulse_down); time.sleep(1)
            set_pulse(pulse_up);   time.sleep(1)
            set_pulse(pulse_down); time.sleep(1)
            pulse_us = pulse_down
        time.sleep(0.02)
finally:
    if old_tty:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)
    release()
    if _HW:
        pi.stop()

print(f"\nFinal config — copy to src/config.py:")
print(f"  SERVO_PULSE_DOWN_US = {pulse_down}")
print(f"  SERVO_PULSE_UP_US   = {pulse_up}")
