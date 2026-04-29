"""
Servo tuning — interactively find correct pulse widths.

Commands (press Enter after each):
  +     increase by 10 us
  -     decrease by 10 us
  ++    increase by 50 us
  --    decrease by 50 us
  u     save current position as UP
  d     save current position as DOWN
  t     test cycle: down → up → down
  g     go to a specific pulse value (prompts for number)
  q     quit and print final config values

Run:
  sudo pigpiod
  source .venv/bin/activate
  python test_servo_tune.py
"""

import time
import pigpio

SERVO_GPIO_1 = 18
SERVO_GPIO_2 = 27

pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon not running. Start with: sudo pigpiod")
    raise SystemExit(1)

pulse_us   = 1400
pulse_down = 1000
pulse_up   = 1800


def set_pulse(us):
    global pulse_us
    pulse_us = max(500, min(2500, us))
    pi.set_servo_pulsewidth(SERVO_GPIO_1, pulse_us)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, pulse_us)
    print(f"  → {pulse_us} us   (saved down={pulse_down}  up={pulse_up})")


def release():
    pi.set_servo_pulsewidth(SERVO_GPIO_1, 0)
    pi.set_servo_pulsewidth(SERVO_GPIO_2, 0)


print("Servo tuner — commands: +  -  ++  --  u  d  t  g  q  (then Enter)")
set_pulse(pulse_us)

try:
    while True:
        cmd = input("> ").strip()
        if cmd == 'q':
            break
        elif cmd == '+':
            set_pulse(pulse_us + 10)
        elif cmd == '-':
            set_pulse(pulse_us - 10)
        elif cmd == '++':
            set_pulse(pulse_us + 50)
        elif cmd == '--':
            set_pulse(pulse_us - 50)
        elif cmd == 'u':
            pulse_up = pulse_us
            print(f"  Saved UP = {pulse_up} us")
        elif cmd == 'd':
            pulse_down = pulse_us
            print(f"  Saved DOWN = {pulse_down} us")
        elif cmd == 't':
            print("  Testing: DOWN → UP → DOWN")
            set_pulse(pulse_down); time.sleep(1.5)
            set_pulse(pulse_up);   time.sleep(1.5)
            set_pulse(pulse_down)
        elif cmd == 'g':
            val = input("  Enter pulse us: ").strip()
            try:
                set_pulse(int(val))
            except ValueError:
                print("  Invalid number")
        else:
            print("  Commands: +  -  ++  --  u  d  t  g  q")
finally:
    release()
    pi.stop()

print(f"\nFinal config — copy to src/config.py:")
print(f"  SERVO_PULSE_DOWN_US = {pulse_down}")
print(f"  SERVO_PULSE_UP_US   = {pulse_up}")
