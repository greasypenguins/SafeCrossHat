import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(15, GPIO.OUT)
GPIO.output(15,GPIO.HIGH)

time.sleep(5)

GPIO.output(15,GPIO.LOW)
GPIO.cleanup()