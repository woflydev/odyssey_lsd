import cv2
import argparse
import tensorflow as tf
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

def pipeline(original_frame, original_angle, SHOW_IMAGES=True):
	cropped, CROPPED_H, CROPPED_W = roi(original_frame)
	result, pot_lines = segments(cropped, 0.13, 20) # used to be 0.1, configures model sensitivity
	pot_line_mask = add_to_mask(pot_lines, (CROPPED_H, CROPPED_W))
	lane_frame, lane_lines = calc_lines(cropped, pot_lines, CROPPED_H, CROPPED_W)
	pot_angle = calc_steering(cropped, lane_lines)
	final_angle = stabilize(original_angle, pot_angle, len(lane_lines))
	preview = heading(lane_frame, final_angle, CROPPED_H, CROPPED_W)
	
	show("original", original_frame, SHOW_IMAGES)
	show("lines", result, SHOW_IMAGES)
	show("line mask", pot_line_mask, SHOW_IMAGES)
	show("preview", preview, SHOW_IMAGES)

	return final_angle

print("INITIALIZING NEURAL NETWORK...")
parser = argparse.ArgumentParser('project_odyssey')
parser.add_argument('--model_path', default='tflite_models/M-LSD_320_tiny_fp32.tflite', type=str, help='path to tflite model')
parser.add_argument('--input_size', default=320, type=int, choices=[512, 320], help='input size')
args = parser.parse_args()

interpreter = tf.lite.Interpreter(model_path=args.model_path)

interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

#IMAGE_W  = int(cap.get(3))  # float `width`
#IMAGE_H = int(cap.get(4))  # float `height`
#angle = 90