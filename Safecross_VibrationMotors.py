import RPi.GPIO as GPIO
import time

"""                           Turning GPIO pin on and off
print("Setting pin numbering")
GPIO.setmode(GPIO.BOARD)

print("Assigning pins")
GPIO.setup(15, GPIO.OUT)

print("pin 15 high")
GPIO.output(15,GPIO.HIGH)

print("sleeping...")
time.sleep(5)

print("pin 15 low")
GPIO.output(15,GPIO.LOW)
GPIO.cleanup()
"""

for x in range(20):
    if x > 5 and x < 15:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(15, GPIO.OUT)
        GPIO.output(15, GPIO.HIGH)
    else:
        GPIO.output(15,GPIO.LOW)
        GPIO.cleanup()
        