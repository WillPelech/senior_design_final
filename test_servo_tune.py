"""
Servo tuner for CONTINUOUS ROTATION servos.

These servos spin, not rotate to a position.
  ~1500 us = stopped (neutral)
  >1500 us = spin one direction
  <1500 us = spin other direction

Goal: find UP pulse, DOWN pulse, and how long to run each.

Controls (type command then Enter):
  w / W    UP pulse   +10 / +50 us
  s / S    UP pulse   -10 / -50 us
  e / E    DOWN pulse +10 / +50 us
  d / D    DOWN pulse -10 / -50 us
  [ / ]    Duration   -0.1 / +0.1 s
  u        Run UP pulse for duration then stop
  n        Run DOWN pulse for duration then stop
  t        Full cycle: DOWN → stop → UP → stop → DOWN → stop
  q        Quit and print final config

Run:
  sudo pigpiod
  source .venv/bin/activate
  python test_servo_tune.py
"""

import time
import pigpio

SERVO_GPIO_LEFT  = 12   # BCM GPIO 12 / Physical Pin 32
SERVO_GPIO_RIGHT = 13   # BCM GPIO 13 / Physical Pin 33

MAX_US = 2500
MIN_US = 500

up_us   = 1700   # starting guess: spin one direction
down_us = 1300   # starting guess: spin other direction
dur_s   = 1.0    # seconds to run before stopping

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running — start with: sudo pigpiod")
    raise SystemExit(1)


def _clamp(us):
    return max(MIN_US, min(MAX_US, us))


def _run(pulse_us, label):
    print(f"  Spinning {label} ({pulse_us} us) for {dur_s:.1f}s...")
    pi.set_servo_pulsewidth(SERVO_GPIO_LEFT,  pulse_us)
    pi.set_servo_pulsewidth(SERVO_GPIO_RIGHT, pulse_us)
    time.sleep(dur_s)
    pi.set_servo_pulsewidth(SERVO_GPIO_LEFT,  0)
    pi.set_servo_pulsewidth(SERVO_GPIO_RIGHT, 0)
    print("  Stopped.")


def _status():
    print(f"  UP={up_us} us   DOWN={down_us} us   duration={dur_s:.1f}s")


print(__doc__)
print("─" * 42)
_status()
print()

try:
    while True:
        cmd = input("> ").strip()
        if cmd == 'q':
            break
        elif cmd == 'w':  up_us = _clamp(up_us + 10);   _status()
        elif cmd == 's':  up_us = _clamp(up_us - 10);   _status()
        elif cmd == 'W':  up_us = _clamp(up_us + 50);   _status()
        elif cmd == 'S':  up_us = _clamp(up_us - 50);   _status()
        elif cmd == 'e':  down_us = _clamp(down_us + 10); _status()
        elif cmd == 'd':  down_us = _clamp(down_us - 10); _status()
        elif cmd == 'E':  down_us = _clamp(down_us + 50); _status()
        elif cmd == 'D':  down_us = _clamp(down_us - 50); _status()
        elif cmd == '[':  dur_s = max(0.1, round(dur_s - 0.1, 1)); _status()
        elif cmd == ']':  dur_s = round(dur_s + 0.1, 1);            _status()
        elif cmd == 'u':  _run(up_us,   "UP")
        elif cmd == 'n':  _run(down_us, "DOWN")
        elif cmd == 't':
            _run(down_us, "DOWN")
            time.sleep(0.5)
            _run(up_us,   "UP")
            time.sleep(0.5)
            _run(down_us, "DOWN")
        else:
            print("  w/s=UP pulse  e/d=DOWN pulse  [/]=duration  u=run UP  n=run DOWN  t=cycle  q=quit")
finally:
    pi.set_servo_pulsewidth(SERVO_GPIO_LEFT,  0)
    pi.set_servo_pulsewidth(SERVO_GPIO_RIGHT, 0)
    pi.stop()

print(f"\nFinal config — copy to src/config.py:")
print(f"  SERVO_PULSE_UP_US   = {up_us}")
print(f"  SERVO_PULSE_DOWN_US = {down_us}")
print(f"  SERVO_TRAVEL_TIME_S = {dur_s}")
