import cv2
import argparse
import tensorflow as tf
import car_client
try:
	from utils.motor_lib.driver import move, off
	DRIVER_INITIALIZED = True
except:
	print("FAILED TO INITIALIZE. RUNNING ANYWAY!")
	DRIVER_INITIALIZED = False
from tools import ( roi,
		   			pwm,
					show,
		   			pred_lines,
		   			calc_lines,
					add_to_mask, 
					calc_steering,
					stabilize,
					heading,
					pred_squares )

VIDEO_SOURCE = 0
BASE_SPEED = 30
SHOW_IMAGES = True

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

angle = 90
print("Starting...")
while True:
		ret, frame = cap.read()
		if ret is None:
			print("no camera feed detected!")
			exit()

		cropped, CROPPED_H, CROPPED_W = roi(frame)
		result, pot_lines = segments(cropped, 0.13, 20) # used to be 0.1, configures model sensitivity
		pot_line_mask = add_to_mask(pot_lines, (CROPPED_H, CROPPED_W))
		lane_frame, lane_lines = calc_lines(cropped, pot_lines, CROPPED_H, CROPPED_W)
		pot_angle = calc_steering(cropped, lane_lines)
		angle = stabilize(angle, pot_angle, len(lane_lines))
		preview = heading(lane_frame, angle, CROPPED_H, CROPPED_W)

		left, right = pwm(BASE_SPEED, angle - 90)

		print(f"Motor Left: {left}, Motor Right: {right}")

		move(left, right) if DRIVER_INITIALIZED else 0 # if motor driver is enabled, drive

		show("original", frame, SHOW_IMAGES)
		show("lines", result, SHOW_IMAGES)
		show("line mask", pot_line_mask, SHOW_IMAGES)
		show("preview", preview, SHOW_IMAGES)

		cv2.imwrite("steer.test.png", preview)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			if DRIVER_INITIALIZED:
				off()
			exit()