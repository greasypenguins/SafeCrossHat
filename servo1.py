import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT) #set board pin 7 as output

p = GPIO.PWM(7,50) #set pin 7 with 50 Hz
p.start(7.5) #set duty cycle of 7.5 for neutral position

try:
        while True: 
                p.ChangeDutyCycle(7.5) #start at duty cylce 7.5 for neutral 90 degrees
                time.sleep(1) #pause time for 1 second
                p.ChangeDutyCycle(12.5) #duty cycle of 12.5 for 180 degrees
                time.sleep(1)
                p.ChangeDutyCycle(2.5) #duty cycle at 2.5 for 0 degrees
                time.sleep(1)
except KeyboardInterrupt:
        GPIO.cleanup()
