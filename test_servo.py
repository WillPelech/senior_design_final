import RPi.GPIO as GPIO
import time

# FS90R continuous rotation servo
# Connect signal wire to physical pin 12 (GPIO 18)
PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

# Use hardware PWM at 50Hz
p = GPIO.PWM(PIN, 50)
p.start(7.5)  # neutral
time.sleep(1)

print('Forward (duty=10)...')
p.ChangeDutyCycle(10)
time.sleep(3)

print('Neutral (duty=7.5)...')
p.ChangeDutyCycle(7.5)
time.sleep(2)

print('Reverse (duty=5)...')
p.ChangeDutyCycle(5)
time.sleep(3)

print('Neutral...')
p.ChangeDutyCycle(7.5)
time.sleep(1)

p.stop()
GPIO.cleanup()
print('Done')
