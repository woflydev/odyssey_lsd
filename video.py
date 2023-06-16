import cv2
import argparse
import numpy as np
from PIL import Image
import tensorflow as tf
from utils import ( roi,
		   							pwm,
		   							pred_lines,
		   							calc_lines,
										add_to_mask, 
										calc_steering,
										stabilize,
										heading,
										pred_squares )

WIDTH_CROP_FACTOR = 1
HEIGHT_CROP_FACTOR = 2
VIDEO_SOURCE = 4
BASE_SPEED = 80

def segments(img_input, score_thr, dist_thr):
	try:
		lines = pred_lines(img_input, interpreter, input_details, output_details, input_shape=[args.input_size, args.input_size], score_thr=score_thr, dist_thr=dist_thr)
		img_output = img_input.copy()
		for line in lines:
			x_start, y_start, x_end, y_end = [int(val) for val in line]
			cv2.line(img_output, (x_start, y_start), (x_end, y_end), [0,255,255], 10)
		#cv2.line(mask, (x_start, y_start), (x_end, y_end), [255,255,255], 10)
	except:
		print("ERROR: no lines found!")
		img_output = img_input.copy()
		lines = []

	return img_output, lines

parser = argparse.ArgumentParser('project_odyssey')
parser.add_argument('--model_path', default='tflite_models/M-LSD_320_tiny_fp32.tflite', type=str, help='path to tflite model')
parser.add_argument('--input_size', default=320, type=int, choices=[512, 320], help='input size')
args = parser.parse_args()

interpreter = tf.lite.Interpreter(model_path=args.model_path)

interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

cap = cv2.VideoCapture(VIDEO_SOURCE)
IMAGE_W  = int(cap.get(3))  # float `width`
IMAGE_H = int(cap.get(4))  # float `height`
IMAGE_CROPPED_W = IMAGE_W // WIDTH_CROP_FACTOR
IMAGE_CROPPED_H = IMAGE_H // HEIGHT_CROP_FACTOR

angle = 0
while True:
	ret, frame = cap.read()
	if ret is None:
		print("no camera feed detected!")
		exit()

	cropped = roi(frame, HEIGHT_CROP_FACTOR, WIDTH_CROP_FACTOR)
	result, pot_lines = segments(cropped, 0.13, 20) #used to be 0.1
	pot_line_mask = add_to_mask(pot_lines, (IMAGE_CROPPED_H, IMAGE_CROPPED_W))
	lane_frame, lane_lines = calc_lines(cropped, pot_lines, IMAGE_CROPPED_H, IMAGE_CROPPED_W)
	pot_angle = calc_steering(cropped, lane_lines)
	angle = stabilize(angle, pot_angle, len(lane_lines))
	preview = heading(lane_frame, angle, IMAGE_CROPPED_H, IMAGE_CROPPED_W)
	
	left, right = pwm(BASE_SPEED, angle - 90)
	
	print(f"Motor Left: {left}, Motor Right: {right}")

	cv2.imshow("original", frame)
	cv2.imshow("lines", result)
	cv2.imshow("line mask", pot_line_mask)
	cv2.imshow("preview", preview)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break