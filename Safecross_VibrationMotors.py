import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT) #set board pin 7 as output

w = width
h = height
on = GPIO.HIGH
off = GPIO.LOW

pos = 0 #duty cycle position

p = GPIO.PWM(7,50) #set pin 7 with 50 Hz
p.start(7.5) #set duty cycle of 7.5 for neutral position

w = width
h = height
on = GPIO.HIGH
off = GPIO.LOW
area = w*h


#Determining vehicle distance
for i, b in enumerate(boxes[0]):
    if CLASSES[class_id] in VEHICLE_CLASSES:
    """if classes [0][i] == 3 or #car
       classes [0][i] == 4 or #motorcycle
       classes [0][i] == 6 or #bus
       classes [0][i] == 8    #truck
    """
    if confidence[0][i]>=0.5:
        #finding mid point of car
        mid_x = (boxes[0][i][1]+boxes[0][i][3])/2
        mid_y = (boxes[0][i][0]+boxes[0][i][2])/2
        #approx_distance based on percentage on what is closes
        approx_distance = round(((1-(boxes[0][i][3] - boxes[0][i][1]))**4),1)
        cv2.putText(image_np,'{}'.format(approx_distance),(int(mid_y*450),int(mid_x*800)),cv2.FONT_HERSEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if approx_distance <0.9:#detecting if distance is less then 0.9
            #motor pins 11 and 13 for both pi's
            GPIO.setup(11,GPIO.OUT) #set board pin 11 as output
            GPIO.output(11, on)#turns on board pin 11
            GPIO.setup(13,GPIO.OUT) #set board pin 13 as output
            GPIO.output(13, off)#turns off board pin 13
        elif approx_distance <0.8:
            GPIO.setup(11,GPIO.OUT) #set board pin 11 as output
            GPIO.output(11, off)#turns off board pin 11
            GPIO.setup(13,GPIO.OUT) #set board pin 13 as output
            GPIO.output(13, on)#turns on board pin 13
            
            if mid_x> 0.4 and mid_x<0.5:
                #warning text display
                cv2.putText(image_np, 'WARNING!!!', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)

    


"""
                TEST Vibration Motors
try:
        while pos = 7.5: 
                p.ChangeDutyCycle(pos) #start at duty cylce 7.5 for neutral 90 degrees
                time.sleep(2) #pause time for 2 second
                
                #if statement for when motor at 90 degrees
                if area >= 250
                    GPIO.output(11, on)
                    time.sleep(2)
                    GPIO.output(13, off)
                elif 
                    GPIO.output(13, on)
                    time.sleep(2)
                    GPIO.output(11, off)
                    
        while pos = 2.5:
                p.ChangeDutyCycle(pos) #duty cycle at 2.5 for 0 degrees
                time.sleep(2)
                if area >= 220
                    GPIO.output(17, on)
                    time.sleep(2)
                    GPIO.output(15, off)
                elif 
                    GPIO.output(15, on)
                    time.sleep(2)
                    GPIO.output(17, off)
except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()


                            Turning on vibraition motors based on w and h
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
except KeyboardInterrupt:
     GPIO.cleanup()
"""


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