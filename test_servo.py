import RPi.GPIO as GPIO
import time

PIN = 18  # GPIO 18 = physical pin 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)
p = GPIO.PWM(PIN, 50)
p.start(0)

try:
    for duty in [2.5, 5.0, 7.5, 10.0, 12.5]:
        print(f'Duty cycle: {duty}%')
        p.ChangeDutyCycle(duty)
        time.sleep(1.5)
finally:
    p.stop()
    GPIO.cleanup()
print('Done')
