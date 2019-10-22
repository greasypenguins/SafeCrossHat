# Run this script on both Pis, one with "1" set and the other with "2"
# set. Pi 1 will control the servo and tell Pi 2 when it has rotated. Pi
# 2 will listen for this signal and take a picture when it can.
# WMH: Still need to implement this difference and signaling

# Imports
import argparse
import cv2
import numpy as np
import os.path
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time

# Plug MOSFET gates into these pins (board numbering):
# BE VERY CAREFUL!
MOTOR_PINS = [11, 13]
SERVO_PIN = 7
P = None

CLASSES = None
COLORS = None
LOOKING_LEFT = True
OUTPUT_VIDEO = False
PI = 0
SAVE_IMAGES = False
VEHICLE_CLASSES = [
    "bicycle",
    "car",
    "motorcycle",
    "bus",
    "truck"
    ]
VEHICLE_IDS = set()
PATH_PREFIX = os.path.dirname(os.path.abspath(__file__))
print(PATH_PREFIX)

def main():
    global CLASSES
    global COLORS
    global OUTPUT_VIDEO
    global P
    global PI
    global SAVE_IMAGES
    
    # Process command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "pi",
        help="Which Pi this is.\n"
        "1: Master Pi controlling the servo motor\n"
        "2: Slave Pi not controlling the servo motor\n"
        )
    parser.add_argument(
        "-v",
        "--video",
        help="Output video feed on screen",
        action="store_true"
        )
    parser.add_argument(
        "-s",
        "--save",
        help="Save pre- and post-processing images in \"temp/pre\" and"
             " \"temp/post\"",
        action="store_true"
        )
    args = parser.parse_args()
    
    if (args.pi != "1") and (args.pi != "2"):
        raise Exception("Pi argument must be \"1\" or \"2\"")
    PI = int(args.pi)
    OUTPUT_VIDEO = args.video
    SAVE_IMAGES = args.save

    print("To stop, press ctrl+c or click on the video output and press"
          " \"q\". It might take a few seconds to shut down the"
          " program.")

    # Setup GPIO pins
    GPIO.setmode(GPIO.BOARD)

    for pin in MOTOR_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    GPIO.setup(SERVO_PIN,GPIO.OUT) # Set servo GPIO pin as PWM output
    P = GPIO.PWM(SERVO_PIN, 50) # 50 Hz PWM
    P.start(7.5) # Set duty cycle of 7.5 for neutral position

    # Initialize the camera
    width = 640
    height = 416
    camera = PiCamera()
    camera.resolution = (width, height)
    camera.framerate = 20
    rawCapture = PiRGBArray(camera, size=(width, height))

    # Allow the camera to warm up
    time.sleep(1)

    # Capture an image
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    if OUTPUT_VIDEO:
        cv2.imshow("Video", image)

    # Setup parameters
    scale = 0.00392
    conf_threshold = 0.5
    nms_threshold = 0.4

    # Setup opencv model
    with open("{}/yolov3.txt".format(PATH_PREFIX), 'r') as f:
        CLASSES = [line.strip() for line in f.readlines()]

    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

    net = cv2.dnn.readNet(
        "{}/yolov3-tiny.weights".format(PATH_PREFIX),
        "{}/yolov3-tiny.cfg".format(PATH_PREFIX)
        )

    output_layers = get_output_layers(net)

    # Setup vehicle classes
    for i, line in enumerate(CLASSES):
        if line in VEHICLE_CLASSES:
            VEHICLE_IDS.add(i)

    if len(VEHICLE_IDS) != len(VEHICLE_CLASSES):
        raise Exception("Could not find the desired classes!")

    # Clear buffer for the next frame
    rawCapture.truncate(0)

    print("Capturing image")

    try:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            rotate_camera()
            
            capture_time = time.time()

            image = rawCapture.array
            if SAVE_IMAGES:
                cv2.imwrite(
                "{}/temp/pre/{}.jpg".format(PATH_PREFIX, str(capture_time)),
                image
                )
            
            # Preprocess image
            blob = cv2.dnn.blobFromImage(
                image,
                scale,
                (416, 416),
                (0, 0, 0),
                True,
                crop=False
                )

            net.setInput(blob)

            # Feed image through neural network
            outs = net.forward(output_layers)

            class_ids = []
            confidences = []
            boxes = []

            # Process output of neural network
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        x = center_x - w / 2
                        y = center_y - h / 2
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])

            indices = cv2.dnn.NMSBoxes(
                boxes,
                confidences,
                conf_threshold,
                nms_threshold
                )

            detected_ids = set()
            for i in indices:
                i = i[0]
                box = boxes[i]
                x = box[0]
                y = box[1]
                w = box[2]
                h = box[3]
                if w * h > 10:
                    detected_ids.add(i)
                if OUTPUT_VIDEO or SAVE_IMAGES:
                    draw_prediction(
                        image,
                        class_ids[i],
                        round(x),
                        round(y),
                        round(x+w),
                        round(y+h)
                        )

            vibrate_motors(detected_ids)
            
            # Output the frame
            if OUTPUT_VIDEO:
                cv2.imshow("Video", image)
            if SAVE_IMAGES:
                cv2.imwrite(
                    "{}/temp/post/{}.jpg".format(PATH_PREFIX, str(capture_time)),
                    image
                    )

            # If the "q" key was pressed, end the program
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                raise Exception(
                    "Exiting because \"q\" was pressed in"
                    " the video output window."
                    )

            # Clear buffer for the next frame
            rawCapture.truncate(0)
            
            print("Capturing image")

    except:
        for motor in range(len(MOTOR_PINS)):
            turn_motor_off(motor)
        time.sleep(0.2)
        GPIO.cleanup()
        camera.close()
        raise
    
    return

def turn_motor_on(motor):
    #WMH: Brandon may modify/remove this
    GPIO.output(MOTOR_PINS[motor], GPIO.HIGH)

    return

def turn_motor_off(motor):
    #WMH: Brandon may modify/remove this
    GPIO.output(MOTOR_PINS[motor], GPIO.LOW)

    return

def get_output_layers(net):
    layer_names = net.getLayerNames()

    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers

def draw_prediction(img, class_id, x, y, x_plus_w, y_plus_h):
    label = str(CLASSES[class_id])

    color = COLORS[class_id]

    cv2.rectangle(
        img,
        (x,y),
        (x_plus_w,y_plus_h),
        color,
        2
        )

    cv2.putText(
        img,
        label,
        (x-10,y-10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2
        )

    return

def vibrate_motors(detected_ids):
    #WMH: Brandon may modify this to take in the heights and widths and
    #WMH: do PWM patterns
    for i in detected_ids:
        print("    Detected {}".format(CLASSES[i]))
        
    detected_vehicle_ids = VEHICLE_IDS.intersection(detected_ids)
    
    if len(detected_vehicle_ids) > 0:
        if LOOKING_LEFT:
            # Vehicle detected on the left
            turn_motor_on(0)
        else:
            # Vehicle detected on the right
            turn_motor_on(1)
    else:
        if LOOKING_LEFT:
            # No vehicles on the left
            turn_motor_off(0)
        else:
            # No vehicles on the right
            turn_motor_off(1)

    return

def rotate_camera():
    global LOOKING_LEFT
    
    if LOOKING_LEFT:
        # Currently set to 90 degrees (left)
        # Move camera to 0 degrees (right)
        set_servo_degrees(19.0)
        LOOKING_LEFT = False
        
    else:
        # Currently set to 0 degrees (right)
        # Move camera to 90 degrees (left)
        set_servo_degrees(94.0)
        LOOKING_LEFT = True

    return

def set_servo_degrees(deg):
    global P

    dc_0   =  2.5 # Duty cycle for   0 degrees
    dc_180 = 12.5 # Duty cycle for 180 degrees

    dc = (deg / 180.0) * (dc_180 - dc_0) + dc_0
    
    P.ChangeDutyCycle(dc)
    
    return

if __name__ == "__main__":
    main()
