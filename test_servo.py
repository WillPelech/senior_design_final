import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# FS90R is a continuous rotation servo
# throttle: 1.0 = full forward, 0.0 = stop, -1.0 = full reverse

print('Forward for 2 seconds...')
kit.continuous_servo[0].throttle = 0.5
time.sleep(2)

print('Stop...')
kit.continuous_servo[0].throttle = 0
time.sleep(1)

print('Reverse for 2 seconds...')
kit.continuous_servo[0].throttle = -0.5
time.sleep(2)

print('Stop.')
kit.continuous_servo[0].throttle = 0
