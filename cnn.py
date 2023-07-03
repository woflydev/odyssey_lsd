#!/usr/bin/python
# THIS FILE IS MAINLY FOR CNN DRIVING. WHILE KEYBOARD DRIVING IS
# POSSIBLE, IT IS NOT RECOMMENDED DUE TO LAG.
import time
import cv2
import math
import numpy as np
import argparse
import logging
from PIL import Image, ImageDraw
from importlib import import_module

try:
	from utils.motor_lib.driver import move, off
except:
	print("MOTOR DRIVER NOT INITIALIZED PROPERLY! RUNNING ANYWAY.")

#---------------------#
# Car Config    			#
#---------------------#
MAX_SPEED = 30
CAMERA_FPS = 30
CAMERA_RESOLUTION = (320, 180)
img_height = 180
img_width = 320
img_channels = 3
VIDEO_SOURCE = 0
VIDEO_FEED = False
MODEL_FILE = "data/models/v0.9"

#---------------------#
# CNN Config 		    	#
#---------------------#
USE_CNN = True

#---------------------#
# Program Config  		#
#---------------------#
frame_id = 0
angle = 0.0
period = 0.05 # sec (=50ms)
interpreter = None
input_index = None
output_index = None
finish = False
logging.basicConfig(level=logging.INFO)

##########################################################
# local functions
##########################################################
def deg2rad(deg):
	return deg * math.pi / 180.0
def rad2deg(rad):
	return 180.0 * rad / math.pi

def stabilize_steering_angle(curr_steering_angle, new):
	max_angle_deviation = 1
	angle_deviation = new - curr_steering_angle
	if abs(angle_deviation) > max_angle_deviation:
		stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
	else:
		stabilized_steering_angle = new
	#logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
	return stabilized_steering_angle

def img_preprocess(image):
	height, _, _ = image.shape
	image = image[int(height/4):,:,:]  # remove top half of the image, as it is not relavant for lane following
	image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)  # Nvidia model said it is best to use YUV color space
	image = cv2.GaussianBlur(image, (3,3), 0)
	image = cv2.resize(image, CAMERA_RESOLUTION) # input image size (200,66) Nvidia model
	image = image / 255 # normalizing, the processed image becomes black for some reason.  do we need this?
	return image

def g_tick():
	t = time.time()
	count = 0
	while True:
		count += 1
		yield max(t + count*period - time.time(),0)

def turn_off():
	#actuator.stop()
	off()

def crop_image(img):
	scaled_img = cv2.resize(img, (max(int(img_height * 4 / 3), img_width), img_height))
	fb_h, fb_w, fb_c = scaled_img.shape
	# print(scaled_img.shape)
	startx = int((fb_w - img_width) / 2)
	starty = int((fb_h - img_height) / 2)
	return scaled_img[starty:starty+img_height, startx:startx+img_width,:]

def preprocess(img):
	if args.pre == "crop":
		img = crop_image(img)
	else:
		img = cv2.resize(img, (img_width, img_height))
		img = (img / 255.).astype(np.float32)
		# Convert to grayscale and readd channel dimension
	if img_channels == 1:
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		img = np.reshape(img, (img_height, img_width, img_channels))
		img = img / 255.
	return img

def overlay_image(l_img, s_img, x_offset, y_offset):
	assert y_offset + s_img.shape[0] <= l_img.shape[0]
	assert x_offset + s_img.shape[1] <= l_img.shape[1]

	l_img = l_img.copy()
	
	for c in range(0, 3):
		l_img[y_offset:y_offset+s_img.shape[0],
		x_offset:x_offset+s_img.shape[1], c] = (
		s_img[:,:,c] * (s_img[:,:,3]/255.0) +
		l_img[y_offset:y_offset+s_img.shape[0],
		x_offset:x_offset+s_img.shape[1], c] * (1.0 - s_img[:,:,3]/255.0))
		return l_img

# takes in value of -90 to 90, with 0 being straight
def angle_to_thrust(speed, theta):
	try:
		theta = ((theta + 180) % 360) - 180  # normalize value to [-180, 180)
		speed = min(max(0, speed), 100) # normalize value to [0, 100]
		v_a = speed * (45 - theta % 90) / 45 # falloff of main motor
		v_b = min(100, 2 * speed + v_a, 2 * speed - v_a) # compensation of other motor
		if theta < -90: return -v_b, -v_a
		if theta < 0:   return -v_a, v_b
		if theta < 90:  return v_b, v_a
		return int([v_a, -v_b])
	except:
			logging.error("Couldn't calculate steering PWM!")

def load_tflite():
		global interpreter
		global input_index
		global output_index
		##########################################################
		# import car's CNN tflite model
		##########################################################
		print ("Loading TFLite: " + MODEL_FILE)
		try:
				# Import TFLite interpreter from tflite_runtime package if it's available.
				from tflite_runtime.interpreter import Interpreter
				interpreter = Interpreter(MODEL_FILE+'.tflite', num_threads=args.ncpu)
		except ImportError:
				# If not, fallback to use the TFLite interpreter from the full TF package.
				import tensorflow as tf
				interpreter = tf.lite.Interpreter(model_path=MODEL_FILE+'.tflite', num_threads=args.ncpu)

		interpreter.allocate_tensors()
		input_index = interpreter.get_input_details()[0]["index"]
		output_index = interpreter.get_output_details()[0]["index"]

##########################################################
# program begins
##########################################################

parser = argparse.ArgumentParser(description='Odyssey NNN Main Program')
parser.add_argument("-c", "--cnn", help="Enable CNN", action="store_true")
parser.add_argument("-n", "--ncpu", help="Number of cores to use.", type=int, default=2)
parser.add_argument("-f", "--hz", help="Control frequnecy", type=int)
parser.add_argument("--fpvvideo", help="Take FPV video of DNN driving", action="store_true")
parser.add_argument("--tflite", help="Use TFLite instead of H5 model.", action="store_true")
parser.add_argument("--pre", help="Preprocessing [resize|crop]", type=str, default="resize")
args = parser.parse_args()

if args.cnn: 
	print("CNN Driver is enabled through flags!")
	USE_CNN = True
if args.hz:
	period = 1.0 / args.hz
	print("New Period: ", period)
if args.fpvvideo:
	FPV_VIDEO = True
	print("FPV video is enabled through flags!")

print("Preprocessing:", args.pre)
print("Tensorflow:", not args.tflite)
print("Performance:", args.ncpu, "cores")
print("CNN Model:", MODEL_FILE, "\n")

##########################################################
# import car's CNN model
##########################################################
if args.tflite:
	#logging.warning("L bozo ur not using tflite. Loading H5 model instead...")
	#import tensorflow.keras as ks
	load_tflite()
	#import keras as ks
	#import tensorflow_model_optimization as tfmot
	#with tfmot.quantization.keras.quantize_scope():
		#model = ks.models.load_model(params.model_file+'.h5')
	#model = ks.models.load_model(params.model_file+'.h5')
else:
	try:
		import tensorflow.keras as ks
	except:
		import keras as ks

	import tensorflow_model_optimization as tfmot
	with tfmot.quantization.keras.quantize_scope():
		model = ks.models.load_model(MODEL_FILE+'.h5')
	#model = ks.models.load_model(params.model_file+'.h5')

	"""try:
			# Import TFLite interpreter from tflite_runtime package if it's available.
			from tflite_runtime.interpreter import Interpreter
			interpreter = Interpreter(params.model_file+'.tflite', num_threads=args.ncpu)
	except ImportError:
			# Import TFLMicro interpreter
			try:
					from tflite_micro_runtime.interpreter import Interpreter
					interpreter = Interpreter(params.model_file+'.tflite')
			except:
					# If all failed, fallback to use the TFLite interpreter from the full TF package.
					import tensorflow as tf
					interpreter = tf.lite.Interpreter(model_path=params.model_file+'.tflite', num_threads=args.ncpu)

	interpreter.allocate_tensors()
	input_index = interpreter.get_input_details()[0]["index"]
	output_index = interpreter.get_output_details()[0]["index"]"""

# initialize car modules
#actuator.init(args.throttle)
cap = cv2.VideoCapture(VIDEO_SOURCE)
#atexit.register(turn_off)

g = g_tick()
start_ts = time.time()

frame_arr = []
angle_arr = []

current_angle = 0
current_speed = 0

# enter main loop
while True:
	try:
		ret, frame = cap.read()
		ts = time.time()

		if VIDEO_FEED == True:
			cv2.imshow('frame', frame)
			cv2.waitKey(1) & 0xFF

		elif USE_CNN == True:
			# machine input
			img = img_preprocess(frame)
			img = np.expand_dims(img, axis=0).astype(np.float32)
			radians = 0
			if args.tflite:
				interpreter.set_tensor(input_index, img)
				interpreter.invoke()
				radians = interpreter.get_tensor(output_index)[0][0]
			else:
				radians = model.predict(img, verbose=0)[0][0]

			new_steering_angle = rad2deg(radians)
			current_angle = stabilize_steering_angle(current_angle, new_steering_angle)

		#current_speed = 80

		if current_speed != 0:
			try:
				pwm = angle_to_thrust(current_speed, current_angle)
				pwm_left = int(pwm[0])
				pwm_right = int(pwm[1])
				move((pwm_left), (pwm_right))
			except:
				pwm_left = 0
				pwm_right = 0
				print("No PWM calculation input!")
		else:
			off()

		# for debugging only
		time.sleep(0.1)
			
		print(f"current_angle: {new_steering_angle}, pwm_left: {pwm_left}, pwm_right: {pwm_right}")
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		cv2.imwrite("webcam.test.png", hsv)

	except KeyboardInterrupt:
		break

turn_off()