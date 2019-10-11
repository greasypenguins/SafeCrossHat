import RPi.GPIO as GPIO
import time

# Use board numbering (NOT BCM!!!)
GPIO.setmode(GPIO.BOARD)

# Plug MOSFET gates into these pins (board numbering):
# BE VERY CAREFUL!
motor_pins = [11, 13, 15, 16]

for pin in motor_pins:
	GPIO.setup(pin, GPIO.OUT)

def turn_motor_on(motor):
	GPIO.output(motor_pins[motor], GPIO.HIGH)

def turn_motor_off(motor):
	GPIO.output(motor_pins[motor], GPIO.LOW)

motor = 0
try:
	while True:
		turn_motor_on(motor)
		time.sleep(1)
		turn_motor_off(motor)
		motor = (motor + 1) % len(motor_pins)

except:
	GPIO.cleanup()
