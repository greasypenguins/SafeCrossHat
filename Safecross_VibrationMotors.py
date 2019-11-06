import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

w = 2
h = 3
area = w*h
a = 6
b = 3
try:
    while True:
        if area == a:
            print("Assigning output to Board Pin 16")
            GPIO.setup(11,GPIO.OUT)
            print("Board Pin 11 turns on")
            GPIO.output(11,GPIO.HIGH)
            print("Stops 3 seconds")
            time.sleep(3)
            print("Board Pin 11 turns off")
            GPIO.output(11,GPIO.LOW)
        elif area < b:
            print("Assigning Board Pin 13")
            GPIO.setup(13,GPIO.OUT)
            print("Board Pin 13 turns on")
            GPIO.output(13,GPIO.HIGH)
            print("Stops 3 seconds")
            time.sleep(3)
            print("Board Pin 13 turns off")
            GPIO.output(13,GPIO.LOW)
except KeyboardInterrupt        
     GPIO.cleanup()






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

#Using pigpio to turn gpio on and off
import pigpio
import time

pi = pigpio.pi()

short = 0.0005
long = 0.00168


pi.hardware_PWM(15, 0, 0)
time.sleep (0.1)
pi.hardware_PWM(15, 38000, 300000)
time.sleep (0.009)
pi.hardware_PWM(15, 0, 0)
time.sleep (0.0045)

pi.hardware_PWM(15, 38000, 300000)
time.sleep (short)
pi.hardware_PWM(15, 0, 0)
time.sleep (short) 

pi.hardware_PWM(15, 38000, 300000)
time.sleep (short)
pi.hardware_PWM(15, 0, 0)
time.sleep (short) 

pi.hardware_PWM(15, 38000, 300000)
time.sleep (short)
pi.hardware_PWM(15, 0, 0)
time.sleep (long)   
"""    