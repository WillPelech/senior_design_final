"""Test both motors together — forward 2s, stop 1s, reverse 2s."""
import sys, os, time
sys.path.insert(0, os.path.dirname(__file__))
from adafruit_motorkit import MotorKit
import board

kit = MotorKit(i2c=board.I2C())

print("Both motors forward...")
kit.motor1.throttle = -0.5
kit.motor2.throttle = -0.5
time.sleep(2)

print("Stop...")
kit.motor1.throttle = 0
kit.motor2.throttle = 0
time.sleep(1)

print("Both motors reverse...")
kit.motor1.throttle = 0.5
kit.motor2.throttle = 0.5
time.sleep(2)

kit.motor1.throttle = 0
kit.motor2.throttle = 0
print("Done.")
