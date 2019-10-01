#https://www.arunponnusamy.com/yolo-object-detection-opencv-python.html

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

def get_output_layers(net):
    
    layer_names = net.getLayerNames()
    
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):

    label = str(classes[class_id])

    color = COLORS[class_id]

    cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)

    cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

print("To exit, click into the image output and press \"q\". It might take a few seconds to shut down the program.")

# Initialize the camera and grab a reference to the raw camera capture
width = 640
height = 480
print("1")
camera = PiCamera()
print("2")
camera.resolution = (width, height)
camera.framerate = 20
print("3")
rawCapture = PiRGBArray(camera, size=(width, height))
print("Width: {}".format(width))
print("Height: {}".format(height))

# Allow the camera to warm up
time.sleep(1)

# Capture an image
camera.capture(rawCapture, format="bgr")
image = rawCapture.array
cv2.imshow("Video", image)

# Set up parameters
scale = 0.00392
conf_threshold = 0.5
nms_threshold = 0.4
classes = None

# Set up opencv model
with open("yolov3.txt", 'r') as f:
    classes = [line.strip() for line in f.readlines()]

COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")

# Clear buffer for the next frame
rawCapture.truncate(0)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	print("Captured image. Processing...")
	image = rawCapture.array
	
	# Detect objects
	blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

	net.setInput(blob)

	outs = net.forward(get_output_layers(net))

	class_ids = []
	confidences = []
	boxes = []

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

	indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

	for i in indices:
		i = i[0]
		box = boxes[i]
		x = box[0]
		y = box[1]
		w = box[2]
		h = box[3]
		draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
	
	print("Done processing. Showing image.")
	
	# Show the frame
	cv2.imshow("Video", image)
	
	# If the 'q' key was pressed, end the program
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break

	# Clear buffer for the next frame
	rawCapture.truncate(0)
	
	print("Capturing new image...")
