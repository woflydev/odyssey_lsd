import cv2
import numpy as np
import logging
import math
import atexit
import time
import datetime
import sys
import faulthandler

try:
	from utils.motor_lib.driver import move, off
	DRIVER_INITIALIZED = True
except:
	print("FAILED TO INITIALIZE. RUNNING ANYWAY!")
	DRIVER_INITIALIZED = False

faulthandler.enable()

BASE_SPEED = 30
RATE_LIMIT = False
RATE_LIMIT_VALUE = 0.08

SHOW_IMAGES = True
STEP_SIZE = 5 # DEFAULT 5 - the less steps, the more accurate the obstacle detection
MIN_DISTANCE = 250 # DEFAULT 250 - the distance to consider an obstacle (the larger the number the closer the obstacle is)
MIN_UTURN_THRESHOLD = 300 # DEFAULT 400 - the distance to consider a u-turn (the larger the number the closer the u-turn has to be)
FEELER_AMOUNT = 5 # MINIMUM OF 5 - the amount of feelers to use (the more feelers the more accurate the obstacle detection)
MIDDLE_ANGLE = 635 # DEFAULT 635 - use the included reference_feeler to determine the middle angle

class B_OpenCV_Driver(object):
	def __init__(self, car=None):
		logging.info('Creating a B_OpenCV_Driver...')
		self.car = car
		self.curr_steering_angle = 90

	def follow_lane(self, frame):
		# Main entry point of the lane follower
		show_image("orig", frame)

		lane_lines, frame = detect_lane(frame)
		final_frame = self.steer(frame, lane_lines)

		return final_frame, self.curr_steering_angle

	def steer(self, frame, lane_lines):
		logging.debug('steering...')
		if len(lane_lines) == 0:
			logging.error('No lane lines detected, nothing to do.')
			return frame

		new_steering_angle = compute_steering_angle(frame, lane_lines)
		self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines))

		left, right = pwm(BASE_SPEED, self.curr_steering_angle - 90)

		if left < 0:
			left = 1
		if right < 0:
			right = 1
		
		if left > 100:
			left = 100
		if right > 100:
			right = 100

		print(f"Left: {int(left)}, Right: {int(right)}")

		# actually write values to motor
		move(int(right), int(left)) if DRIVER_INITIALIZED else 0 #motors are wired wrong way around lol

		print(f"Calculated Steering Angle: {self.curr_steering_angle}")

		curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
		show_image("heading", curr_heading_image)

		return curr_heading_image

############################
# Frame processing steps
############################
def detect_lane(frame):
	logging.debug('detecting lane lines...')

	edges = detect_edges(frame)
	show_image('edges', edges)

	cropped_edges = region_of_interest(edges)
	show_image('edges cropped', cropped_edges)

	line_segments = detect_line_segments(cropped_edges)
	line_segment_image = display_lines(frame, line_segments)
	show_image("line segments", line_segment_image)

	lane_lines = average_slope_intercept(frame, line_segments)
	lane_lines_image = display_lines(frame, lane_lines)
	show_image("lane lines", lane_lines_image)

	return lane_lines, lane_lines_image

def detect_edges(frame):
	# filter for blue lane lines
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	show_image("hsv", hsv)

	#lower_blue = np.array([93, 132, 0]) # changed this to different colour for home testing
	#upper_blue = np.array([113, 255, 255])

	#lower_yellow = np.array([0, 0, 255])
	#upper_yellow = np.array([53, 76, 255])

	LOWER_MASK = np.array([0, 62, 0], np.uint8)
	UPPER_MASK = np.array([179, 255, 124], np.uint8)
	mask = cv2.inRange(hsv, LOWER_MASK, UPPER_MASK)

	#mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	#mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

	#show_image("blue mask", mask_blue)
	#show_image("yellow mask", mask_yellow)
	show_image("the thing idk", mask)

	#combined_mask = cv2.bitwise_or(mask_blue, mask_yellow)

	#cv2.imwrite("camera.test.png", combined_mask)

	# detect edges
	#edges = cv2.Canny(combined_mask, 200, 400)
	edges = cv2.Canny(mask, 200, 400)

	return edges

def detect_edges_old(frame):
	# filter for blue lane lines
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	show_image("hsv", hsv)
	for i in range(16):
		lower_blue = np.array([30, 16 * i, 0])
		upper_blue = np.array([150, 255, 255])
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		show_image("blue mask Sat=%s" % (16* i), mask)


	#for i in range(16):
		#lower_blue = np.array([16 * i, 40, 50])
		#upper_blue = np.array([150, 255, 255])
		#mask = cv2.inRange(hsv, lower_blue, upper_blue)
		#show_image("blue mask hue=%s" % (16* i), mask)

		# detect edges
	edges = cv2.Canny(mask, 200, 400)

	return edges


def region_of_interest(canny):
	height, width = canny.shape
	mask = np.zeros_like(canny)

	# only focus bottom half of the screen

	polygon = np.array([[
		(0, height * 1 / 2),
		(width, height * 1 / 2),
		(width, height),
		(0, height),
	]], np.int32)

	cv2.fillPoly(mask, polygon, 255)
	show_image("mask", mask)
	masked_image = cv2.bitwise_and(canny, mask)
	return masked_image


def detect_line_segments(cropped_edges):
	# tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
	rho = 1  # precision in pixel, i.e. 1 pixel
	angle = np.pi / 180  # degree in radian, i.e. 1 degree
	min_threshold = 10  # minimal of votes #default 10
	line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=10,
									maxLineGap=2) #minlinelength=8, maxlinegap=4

	if line_segments is not None:
		for line_segment in line_segments:
			logging.debug('detected line_segment:')
			logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

	return line_segments

def average_slope_intercept(frame, line_segments):
	"""
	This function combines line segments into one or two lane lines
	If all line slopes are < 0: then we only have detected left lane
	If all line slopes are > 0: then we only have detected right lane
	"""
	lane_lines = []
	if line_segments is None:
		logging.info('No line_segment segments detected')
		return lane_lines

	height, width, _ = frame.shape
	left_fit = []
	right_fit = []

	boundary = 1/3
	left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
	right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

	for line_segment in line_segments:
		for x1, y1, x2, y2 in line_segment:
			if x1 == x2:
				logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
				continue
			try: # changed
				fit = np.polyfit((x1, x2), (y1, y2), 1)
			except np.RankWarning:
				fit = None
			slope = fit[0]
			intercept = fit[1]
			if slope < 0:
				if x1 < left_region_boundary and x2 < left_region_boundary:
					left_fit.append((slope, intercept))
			else:
				if x1 > right_region_boundary and x2 > right_region_boundary:
					right_fit.append((slope, intercept))

	left_fit_average = np.average(left_fit, axis=0)
	if len(left_fit) > 0:
		lane_lines.append(make_points(frame, left_fit_average))

	right_fit_average = np.average(right_fit, axis=0)
	if len(right_fit) > 0:
		lane_lines.append(make_points(frame, right_fit_average))

	logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

	return lane_lines


def compute_steering_angle(frame, lane_lines):
	""" Find the steering angle based on lane line coordinate
		We assume that camera is calibrated to point to dead center
	"""
	if len(lane_lines) == 0:
		logging.info('No lane lines detected, do nothing')
		return -90

	height, width, _ = frame.shape
	if len(lane_lines) == 1:
		logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
		x1, _, x2, _ = lane_lines[0][0]
		x_offset = x2 - x1
	else:
		_, _, left_x2, _ = lane_lines[0][0]
		_, _, right_x2, _ = lane_lines[1][0]
		camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
		mid = int(width / 2 * (1 + camera_mid_offset_percent))
		x_offset = (left_x2 + right_x2) / 2 - mid

	# find the steering angle, which is angle between navigation direction to end of center line
	y_offset = int(height / 2)

	angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
	angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
	steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

	logging.debug('new steering angle: %s' % steering_angle)
	return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=4):
	"""
	Using last steering angle to stabilize the steering angle
	This can be improved to use last N angles, etc
	if new angle is too different from current angle, only turn by max_angle_deviation degrees
	"""
	if num_of_lane_lines == 2 :
		# if both lane lines detected, then we can deviate more
		max_angle_deviation = max_angle_deviation_two_lines
	else :
		# if only one lane detected, don't deviate too much
		max_angle_deviation = max_angle_deviation_one_lane
	
	angle_deviation = new_steering_angle - curr_steering_angle
	if abs(angle_deviation) > max_angle_deviation:
		stabilized_steering_angle = int(curr_steering_angle
										+ max_angle_deviation * angle_deviation / abs(angle_deviation))
	else:
		stabilized_steering_angle = new_steering_angle
	logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
	return stabilized_steering_angle


############################
# Utility Functions
############################
def pwm(speed, theta):
	try:
		theta = ((theta + 180) % 360) - 180  # normalize value to [-180, 180)
		speed = min(max(0, speed), 100)              # normalize value to [0, 100]
		v_a = speed * (45 - theta % 90) / 45          # falloff of main motor
		v_b = min(100, 2 * speed + v_a, 2 * speed - v_a)  # compensation of other motor
		if theta < -90: return -v_b, -v_a
		if theta < 0:   return -v_a, v_b
		if theta < 90:  return v_b, v_a
		return [v_a, -v_b]
	except:
			print('error')

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
	line_image = np.zeros_like(frame)
	if lines is not None:
		for line in lines:
			for x1, y1, x2, y2 in line:
				cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
	line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
	return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
	heading_image = np.zeros_like(frame)
	height, width, _ = frame.shape

	# figure out the heading line from steering angle
	# heading line (x1,y1) is always center bottom of the screen
	# (x2, y2) requires a bit of trigonometry

	# Note: the steering angle of:
	# 0-89 degree: turn left
	# 90 degree: going straight
	# 91-180 degree: turn right 
	steering_angle_radian = steering_angle / 180.0 * math.pi
	x1 = int(width / 2)
	y1 = height
	x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
	y2 = int(height / 2)

	cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
	heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

	return heading_image


def length_of_line_segment(line):
	x1, y1, x2, y2 = line
	return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def show_image(title, frame, show=SHOW_IMAGES):
	if show:
		cv2.imshow(title, frame)


def make_points(frame, line):
	height, width, _ = frame.shape
	slope, intercept = line
	y1 = height  # bottom of the frame
	y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

	# bound the coordinates within the frame
	x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
	x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
	return [[x1, y1, x2, y2]]


############################
# Test Functions
############################
def test_photo(file):
	land_follower = B_OpenCV_Driver()
	frame = cv2.imread(file)
	combo_image = land_follower.follow_lane(frame)
	show_image('final', combo_image, True)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


def test_video(video_file):
	lane_follower = B_OpenCV_Driver()
	cap = cv2.VideoCapture(video_file)

	# skip first second of video.
	for i in range(60):
		_, frame = cap.read()

	#video_type = cv2.VideoWriter_fourcc(*'XVID')
	#video_overlay = cv2.VideoWriter("%s_overlay.avi" % (video_file), video_type, 20.0, (320, 240))
	
	try:
		i = 0
		while cap.isOpened():
			atexit.register(off) if DRIVER_INITIALIZED else 0
			_, frame = cap.read()
			print('frame %s' % i )
			combo_image, currentAngle = lane_follower.follow_lane(frame)

			cv2.imwrite("camera.test.png", combo_image)

			# hsv for purple boxes
			#lower_purple = np.array([141, 59, 0]) <- campusData.mp4
			#upper_purple = np.array([179, 255, 255])

			# school track with new camera
			lower_purple = np.array([121, 96, 0])
			upper_purple = np.array([179, 255, 255])

			# hsv for blue and yellow
			#lower_blue = np.array([0, 76, 158])
			#upper_blue = np.array([179, 255, 255])
			
			#lower_yellow = np.array([141, 59, 0])
			#upper_yellow = np.array([179, 255, 255])

			obstacle_frame = frame.copy()
			uturn_frame = frame.copy()
			#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			#obstacle_mask = cv2.inRange(hsv, lower_purple, upper_purple)
			#obstacle_blur = cv2.bilateralFilter(obstacle_mask, 9, 40, 40)
			#obstacle_edges = cv2.Canny(obstacle_blur, 50, 100)

			#uturn_mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
			#uturn_mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)
			#uturn_mask = cv2.bitwise_or(uturn_mask_b, uturn_mask_y)
			#uturn_blur = cv2.bilateralFilter(uturn_mask_b, 9, 40, 40)
			#uturn_edges = cv2.Canny(uturn_blur, 50, 100)

			# overrides for the current angle
			#adj_ut, uturn_frame = ovr.detect_uturns(uturn_edges, uturn_frame)
			#adj_ob, obstacle_frame = ovr.detect_obstacles(obstacle_edges, obstacle_frame)

			#currentAngle += adj_ut + adj_ob

			#currentAngle += adj_ob
			#print(f"Angle Adjustment: {adj_ob}")

			#time.sleep(0.01)
			#pwm = throttle_angle_to_thrust(BASE_SPEED, currentAngle - 90)
			#print(f"Left: {pwm[0]}, Right: {pwm[1]}")

			# actually write values to motor
			#driver.move(pwm[0], pwm[1])
			
			#cv2.imwrite("%s_%03d_%03d.png" % (video_file, i, lane_follower.curr_steering_angle), frame)
			#cv2.imwrite("%s_overlay_%03d.png" % (video_file, i), combo_image)
			#video_overlay.write(combo_image)
			#cv2.imshow("Road with Lane line", combo_image)

			i += 1
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			
			time.sleep(RATE_LIMIT_VALUE) if RATE_LIMIT else 0
	finally:
		cap.release()
		#video_overlay.release()
		cv2.destroyAllWindows()


if __name__ == '__main__':
	logging.basicConfig(level=logging.ERROR)
	#test_photo('/home/pi/DeepPiCar/driver/data/video/car_video_190427_110320_073.png')
	#test_photo(sys.argv[1])
	#test_video(gstreamer_pipeline(flip_method=2))
	test_video(4)
	#test_video("https://192.168.227.41:8080")
	#test_video("data/TestTrack.mp4")
	