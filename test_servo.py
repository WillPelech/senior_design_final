import RPi.GPIO as GPIO
import time

pins = [21, 22]
GPIO.setmode(GPIO.BCM)

for pin in pins:
    print(f'Testing GPIO {pin}...')
    GPIO.setup(pin, GPIO.OUT)
    p = GPIO.PWM(pin, 50)
    p.start(5)
    time.sleep(1)
    p.ChangeDutyCycle(10)
    time.sleep(1)
    p.stop()
    GPIO.cleanup(pin)

print('Done')
