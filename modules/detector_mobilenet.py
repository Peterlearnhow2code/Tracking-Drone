import jetson.inference
import jetson.utils
import cv2
import numpy as np

net = None
camera = None

def initialize_detector():
	global net, camera
	net = jetson.inference.detectNet("ssd-inception-v2", threshold=0.5)
	camera = jetson.utils.videoSource("csi://0", argv=[
		'--input-width=1920',
		'--input-height=1080',
		'--input-flip=rotate-180',
	])     
	print("camera and detector initialized")

def get_image_size():
	return camera.GetWidth(), camera.GetHeight()

def close_camera():
	camera.Close()

def get_detections():
	person_detections = []
	img = camera.Capture()
	detections = net.Detect(img)
	for detection in detections:
		if detection.ClassID == 1: #remove unwanted classes
			person_detections.append(detection)
	fps = net.GetNetworkFPS()

	return person_detections, fps, jetson.utils.cudaToNumpy(img)

