import RPi.GPIO as GPIO
import time

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