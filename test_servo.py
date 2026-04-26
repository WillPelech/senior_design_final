import RPi.GPIO as GPIO
import time

PIN = 18  # GPIO 18 = physical pin 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)
p = GPIO.PWM(PIN, 50)
p.start(0)

try:
    print('Running at 2.5% for 3 seconds...')
    p.ChangeDutyCycle(2.5)
    time.sleep(3)
    print('Running at 7.5% (neutral) for 3 seconds...')
    p.ChangeDutyCycle(7.5)
    time.sleep(3)
    print('Running at 12.5% for 3 seconds...')
    p.ChangeDutyCycle(12.5)
    time.sleep(3)
finally:
    p.stop()
    GPIO.cleanup()
print('Done')
