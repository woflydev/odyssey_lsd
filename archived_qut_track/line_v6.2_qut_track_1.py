import cv2
import numpy as np
import time
import math

DRIVER_INITIALIZED = False
try:
	from utils.motor_lib.driver import move, off, brake
	DRIVER_INITIALIZED = True
except:
	print("MOTOR DRIVER NOT INITIALIZED! RUNNING ANYWAY...")

VIDEO_SOURCE = 0

SHOW_IMAGES = False
WRITE_IMAGES = False
BASE_SPEED = 40 #32
BOOST_SPEED = 95
BOOST_ANGLE = 5

BLUR_KERNEL = 5
OBSTACLE_BLUR = 30
# for testing only [0, 62, 0], [179, 255, 124]
# LOW_BLUE = [101, 106, 130]
# HIGH_BLUE = [179, 255, 255]

# LOW_YELLOW = [0, 62, 0] #enochs house
# HIGH_YELLOW = [179, 255, 124]

'''LOW_BLUE = [89, 174, 174] #school track morning
HIGH_BLUE = [122, 255, 255]'''

#LOW_YELLOW = [0, 146, 0] #ermias house
#HIGH_YELLOW = [179, 210, 255]

'''LOW_YELLOW = [0, 0, 219] #school track morning
HIGH_YELLOW = [100, 53, 255]'''

#LOW_BLUE = [83, 60, 83]
#HIGH_BLUE = [109, 255, 255]

#LOW_YELLOW = [0, 0, 157] #school track
#HIGH_YELLOW = [179, 57, 255]

LOW_BLUE = [63, 145, 89] # qut track
HIGH_BLUE = [137, 255, 255]

LOW_YELLOW = [26, 128, 125] # qut track
HIGH_YELLOW = [34, 255, 255]

LOW_PURPLE = [117, 100, 0]
HIGH_PURPLE = [174, 255, 205]

LOW_GREEN = [38, 163, 182]
HIGH_GREEN = [43, 255, 255]
GREEN_THRESHOLD = 2000

def pwm(speed, theta):
	try:
		theta = ((theta + 180) % 360) - 180  # normalize value to [-180, 180)
		speed = min(max(0, speed), 100)              # normalize value to [0, 100]
		v_a = speed * (45 - theta % 90) / 45          # falloff of main motor
		v_b = min(100, 2 * speed + v_a, 2 * speed - v_a)  # compensation of other motor
		if theta < -90: return -v_b, -v_a
		if theta < 0:   return -v_a, v_b
		if theta < 90:  return v_b, v_a
		return [int(v_a), int(-v_b)]
	except:
		print('Unable to calculate PWM! (Most commonly from division by zero)')

def stabilize(current, new, num_lanes, max_confident_deviation=0.5,max_unsure_deviation=0.25):
	"""
	Using last steering angle to stabilize the steering angle
	This can be improved to use last N angles, etc
	if new angle is too different from current angle, only turn by max_angle_deviation degrees
	"""

	#scale = (40 / BASE_SPEED) ** 2

	if num_lanes == 2:
		# if both lane lines detected, then we can deviate more
		max_angle_deviation = max_confident_deviation
	elif num_lanes == 1:
		# if only one lane detected, don't deviate too much
		max_angle_deviation = max_unsure_deviation
	else:
		max_angle_deviation = 2
	#elif num_lanes == 0:
		#max_angle_deviation = 100
	
	angle_deviation = new - current
	#print(f"Angle deviation: {angle_deviation}")
	if abs(angle_deviation) > max_angle_deviation:
		stabilized_steering_angle = int(current
										+ max_angle_deviation * angle_deviation / abs(angle_deviation))
	else:
		stabilized_steering_angle = new
	print('INFO: Proposed angle: %s, stabilized angle: %s' % (new, stabilized_steering_angle))
	return stabilized_steering_angle

def detect_line_segments(cropped_edges):
	# tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
	rho = 1  # precision in pixel, i.e. 1 pixel
	angle = np.pi / 180  # degree in radian, i.e. 1 degree
	min_threshold = 10  # minimal of votes #default 10
	line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=10,
									maxLineGap=2) #minlinelength=8, maxlinegap=4

	'''if line_segments is not None:
		for line_segment in line_segments:
			logging.debug('detected line_segment:')
			logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))
'''
	return line_segments

def slope_intercepts(line_segments):
	"""
	This function combines line segments into one or two lane lines
	If all line slopes are < 0: then we only have detected left lane
	If all line slopes are > 0: then we only have detected right lane
	"""
	lane_line = []
	if line_segments is None:
		return lane_line

	fit = []

	for line_segment in line_segments:
		for x1, y1, x2, y2 in line_segment:
			if x1 == x2:
				#logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
				continue
			try: # changed
				tmpFit = np.polyfit((x1, x2), (y1, y2), 1)
			except np.RankWarning:
				tmpFit = None
			slope = tmpFit[0]
			intercept = tmpFit[1]
			fit.append((slope, intercept, length_of_line_segment((x1, y1, x2, y2))))

	#fit_average = np.average(fit, axis=0)

	return fit

def detect_uturn(line_segments):
	totalLen = 0
	for (slope,_,length) in line_segments:
		if abs(slope) < horizontalSlopeThreshold:
			totalLen += length 
		if totalLen >= horizontalLengthThreshold:
			return True
	return False

def show(window_name, frame, should_write=WRITE_IMAGES):
	if SHOW_IMAGES:
		cv2.imshow(window_name, frame)
	if should_write:
		cv2.imwrite(f"camera.{window_name}.test.png", frame)

def heading(frame, angle):
		heading_image = np.zeros_like(frame)
		height, width, _ = frame.shape
		radians = 0
		if angle is None:
				radians = math.pi / 2
		else:
				radians = angle / 180.0 * math.pi
		x1 = int(width / 2)
		y1 = height
		x2 = 0
		y2 = int(height / 2)
		if radians == 0:
			x2 = 0
			y2 = height
		elif radians == math.pi:
			x2 = width
			y2 = height
		else:
			x2 = int(x1 - height / 2 / math.tan(radians))

		cv2.line(heading_image, (x1, y1), (x2, y2), (0, 0, 255), 10)
		heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
		return heading_image

def avg(numbers, power=1):
	total = 0
	for number in numbers:
		total += number ** power

	return (total / len(numbers)) **(1/power)

def weighted_avg(numbers, values):
		total = 0
		for i in range(len(numbers)):
				total += numbers[i] * values[i]
		return total / sum(values)

def clamp(n, bounds):
		if n < bounds[0]:
				return bounds[0]
		elif n > bounds[1]:
				return bounds[1]
		else:
				return n
		
def length_of_line_segment(line):
	x1, y1, x2, y2 = line
	return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def boost(speed, t):
	move(speed, speed)
	time.sleep(t)

cap = cv2.VideoCapture(VIDEO_SOURCE)

previousYellowAngle = None
previousBlueAngle = None
blueLeft = False
angle = 90
cutoffConstant = 2/3
cutoffMin = 1/3
cutoffMax = 3/4

# 1/16 good for 40 BASE_SPEED
fracOffset = 1/16
horizontalSlopeThreshold = 0.15
horizontalLengthThreshold = 125
cannyMin = 200
cannyMax = 400
uTurnForwardSpeed = 1.3
uTurnBackSpeed = -0.25
otherNone = False
overrideArea = 20

obstacleThreshold = 6000
obstacleWeight = 0.25

finalAngleOffset = 0

contourColor = (0, 0, 255)
obstacleColor = (255, 0, 0)
lineColor = (0, 255, 0)
circleColor = (255, 255, 255)
circleRadius = 5
lineThickness = 3

blueAngle = None
yellowAngle = None

input("PRESS ENTER TO START!")

boost(BOOST_SPEED, 0.2)

startTime = time.time()

# begin main program
try:
	while True:
			error = 0
			ret, frame = cap.read()

			if ret:
					originalFrame = np.copy(frame)
					frame = cv2.blur(originalFrame, (BLUR_KERNEL, BLUR_KERNEL))
					contourFrame = np.copy(frame)
					obstacleFrame = cv2.blur(originalFrame, (OBSTACLE_BLUR, OBSTACLE_BLUR))

					low_b = np.uint8(LOW_BLUE)
					high_b = np.uint8(HIGH_BLUE)

					low_y = np.uint8(LOW_YELLOW)
					high_y = np.uint8(HIGH_YELLOW)

					low_p = np.uint8(LOW_PURPLE)
					high_p = np.uint8(HIGH_PURPLE)

					low_g = np.uint8(LOW_GREEN)
					high_g = np.uint8(HIGH_GREEN)

					hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
					obstacleHSV = cv2.cvtColor(obstacleFrame, cv2.COLOR_BGR2HSV)

					blueMask = cv2.inRange(hsv, low_b, high_b)
					yellowMask = cv2.inRange(hsv, low_y, high_y)
					obstacleMask = cv2.inRange(obstacleHSV, low_p, high_p)
					greenMask = cv2.inRange(hsv, low_g, high_g)

					# Only focuses on the bottom half or section of the screen as determined by cutoff
					bottomLeft = np.zeros_like(blueMask)
					bottomRight = np.zeros_like(yellowMask)

					cutoff = clamp(cutoffConstant * BASE_SPEED / 30, (cutoffMin, cutoffMax))

					bottomLeft = cv2.rectangle(bottomLeft, (0, bottomLeft.shape[0]), (round(bottomLeft.shape[1] * cutoff), round(bottomLeft.shape[0] * (1 - cutoff))), 255, -1)
					bottomRight = cv2.rectangle(bottomRight, (bottomRight.shape[1], bottomRight.shape[0]), (round(bottomRight.shape[1] * (1 - cutoff)), round(bottomRight.shape[0] * (1 - cutoff))), 255, -1)

					blueMask = cv2.bitwise_and(blueMask, blueMask, mask=bottomLeft if blueLeft else bottomRight)
					yellowMask = cv2.bitwise_and(yellowMask, yellowMask, mask=bottomRight if blueLeft else bottomLeft)
	

					blueContours, hierarchy = cv2.findContours(blueMask, 1, cv2.CHAIN_APPROX_NONE) 
					yellowContours, hierarchy = cv2.findContours(yellowMask, 1, cv2.CHAIN_APPROX_NONE)
					# Used simple here to save memory
					obstacleContours, hierarchy = cv2.findContours(obstacleMask, 1, cv2.CHAIN_APPROX_SIMPLE)

					finishContours, hierarchy = cv2.findContours(greenMask, 1, cv2.CHAIN_APPROX_NONE)

					leftEndPoint = (round(frame.shape[1] * fracOffset), frame.shape[0])
					rightEndPoint = (round((1 - fracOffset) * frame.shape[1]), frame.shape[0])

					previousBlueAngle = blueAngle
					previousYellowAngle = yellowAngle

					blueCentre = [0, 0]
					yellowCentre = [0, 0]
					obstacleCentre = [0, 0]

					blueEndPoint = [0, 0]
					yellowEndPoint = [0, 0]
					obstacleEndPoint = [0, 0]

					if len(blueContours) > 0:
							c_b = max(blueContours, key=cv2.contourArea)
							#print(c_b.shape)
							M = cv2.moments(c_b)
							if M["m00"] != 0:
									blueCentre[0] = int(M['m10']/M['m00'])
									blueCentre[1] = int(M['m01']/M['m00'])
									blueEndPoint = leftEndPoint if blueLeft else rightEndPoint
									blueAngle = 180 - round(np.arctan2(blueEndPoint[1] - blueCentre[1], blueCentre[0] - blueEndPoint[0]) * 180 / np.pi)
									cv2.drawContours(contourFrame, [c_b], 0, contourColor, lineThickness)
									cv2.circle(contourFrame, (blueCentre[0],blueCentre[1]), circleRadius, circleColor, -1)
									cv2.line(contourFrame, (blueCentre[0], blueCentre[1]), (round(blueEndPoint[0]), blueEndPoint[1]), lineColor, lineThickness)     
									#print(f"Blue steering angle: {blueAngle} degrees")
					else:
							blueAngle = None

					if len(yellowContours) > 0:
							c_y = max(yellowContours, key=cv2.contourArea)
							M = cv2.moments(c_y)
							if M["m00"] != 0:
									yellowCentre[0] = int(M['m10']/M['m00'])
									yellowCentre[1] = int(M['m01']/M['m00']) 
									yellowEndPoint = rightEndPoint if blueLeft else leftEndPoint
									yellowAngle = 180 - round(np.arctan2(yellowEndPoint[1] - yellowCentre[1], yellowCentre[0] - yellowEndPoint[0]) * 180 / np.pi) 
									cv2.drawContours(contourFrame, [c_y], 0, contourColor, lineThickness)
									cv2.circle(contourFrame, (yellowCentre[0],yellowCentre[1]), circleRadius, circleColor, -1)
									cv2.line(contourFrame, (yellowCentre[0], yellowCentre[1]), (round(yellowEndPoint[0]), yellowEndPoint[1]), lineColor, lineThickness)         
									#print(f"Yellow steering angle: {yellowAngle} degrees")
					else:
						yellowAngle = None

					obstacleDetected = False
					obstacleLeft = False

					if len(finishContours) > 0:
						finish = max(finishContours, key=cv2.contourArea)
						if cv2.contourArea(finish) > GREEN_THRESHOLD:
							print("ZOOMING TO FINISH LINE!")
							move(99, 99)
							time.sleep(0.2)
							brake()

							endTime = time.time()
							print(
							f'\nSTART TIME: {startTime}\nEND TIME: {endTime}\nTOTAL TIME: {round(endTime - startTime)}s'
							)

							input("PRESS ENTER TO CONTINUE TO NEXT LAP!")

							startTime = time.time()

							boost(BOOST_SPEED, 0.2)

							continue
			

					if len(obstacleContours) > 0:
						obstacle = max(obstacleContours, key=cv2.contourArea)
						if (cv2.contourArea(obstacle) > obstacleThreshold):
							obstacleDetected = True
							M = cv2.moments(obstacle)
							if M["m00"] != 0:
								obstacleCentre[0] = int(M['m10']/M['m00'])
								obstacleCentre[1] = int(M['m01']/M['m00'])
								sideLengthHalved = round(math.sqrt(cv2.contourArea(obstacle)) /2)
								cv2.drawContours(contourFrame, [obstacle], 0, obstacleColor, lineThickness)

								if obstacleCentre[0] > frame.shape[1] / 2:
										obstacleEndPoint = rightEndPoint
										if blueLeft:
											blueAngle = 180 - round(np.arctan2(obstacleEndPoint[1] - obstacleCentre[1], obstacleCentre[0] - obstacleEndPoint[0] - sideLengthHalved) * 180 / np.pi)
										else:
											yellowAngle = 180 - round(np.arctan2(obstacleEndPoint[1] - obstacleCentre[1], obstacleCentre[0] - obstacleEndPoint[0] - sideLengthHalved) * 180 / np.pi)
										obstacleCentre[0] -= sideLengthHalved
								else:
									obstacleLeft = True
									obstacleEndPoint = leftEndPoint
									if blueLeft:
										yellowAngle = 180 - round(np.arctan2(obstacleEndPoint[1] - obstacleCentre[1], obstacleCentre[0] - obstacleEndPoint[0] + sideLengthHalved) * 180 / np.pi)
									else:
										blueAngle = 180 - round(np.arctan2(obstacleEndPoint[1] - obstacleCentre[1], obstacleCentre[0] - obstacleEndPoint[0] + sideLengthHalved) * 180 / np.pi)
									obstacleCentre[0] += sideLengthHalved
								cv2.circle(contourFrame, (obstacleCentre[0],obstacleCentre[1]), circleRadius, circleColor, -1)
								cv2.line(contourFrame, (obstacleCentre[0], obstacleCentre[1]), (obstacleEndPoint[0], obstacleEndPoint[1]), lineColor, lineThickness)

					print(f"Blue angle: {blueAngle}, Yellow angle: {yellowAngle}")
					if obstacleDetected and blueAngle is not None and yellowAngle is not None:
						if obstacleLeft:
							angle = weighted_avg([blueAngle, yellowAngle], [(1 - obstacleWeight) if blueLeft else obstacleWeight, obstacleWeight if blueLeft else (1 - obstacleWeight)])
						else:
							angle = weighted_avg([blueAngle, yellowAngle], [obstacleWeight if blueLeft else (1 - obstacleWeight), (1 - obstacleWeight) if blueLeft else obstacleWeight])
					else:
						if blueAngle is None and yellowAngle is not None:
								if previousYellowAngle is not None:
										angle = stabilize(yellowAngle, previousYellowAngle, 1)
										print("stabilizing yellow.")
								else:
										angle = yellowAngle
										print("not stabilizing yellow!")
						elif yellowAngle is None and blueAngle is not None:
								if previousBlueAngle is not None:
										angle = stabilize(blueAngle, previousBlueAngle, 1)
										print("stabilizing blue.")
								else:
										angle = blueAngle
										print("not stabilizing blue!")
						elif blueAngle is not None and yellowAngle is not None:
								if previousBlueAngle is not None and previousYellowAngle is not None:
										angle = stabilize(avg([blueAngle, yellowAngle]), avg([previousBlueAngle, previousYellowAngle]), 2)
										print("stabilizing both!")
								else:
										angle = avg([blueAngle, yellowAngle])
										print("not stabilizing anything.")
						else:
								print("give up lol")
								angle = 90
					#print(f"Previous angles: {previousBlueAngle}, {previousYellowAngle}")
					# Adds obstacle avoidance
					#angle += finalAngleOffset

					#angleLimit = 20
					#angle = clamp(angle, [angleLimit, 180 - angleLimit])
					
					heading_img = heading(contourFrame, angle)

					show("Blue Mask", blueMask)
					show("Yellow Mask", yellowMask)
					show("Obstacle Mask", obstacleMask)
					show("Finish Mask", greenMask)
					show("Frame", frame)
					show("Contours", contourFrame)
					show("Heading", heading_img, True)

					print(f"Steering angle: {angle} degrees")

					left, right = pwm(BASE_SPEED if abs(angle - 90) > BOOST_ANGLE else BOOST_SPEED, angle - 90)

					'''# U-turn code
					blueEdges = cv2.Canny(blueMask, cannyMin, cannyMax)
					yellowEdges = cv2.Canny(yellowMask, cannyMin, cannyMax)

					blueLines = detect_line_segments(blueEdges)
					yellowLines = detect_line_segments(yellowEdges)

					blueFit = slope_intercepts(blueLines)
					yellowFit = slope_intercepts(yellowLines)

					show("Blue Edges", blueEdges)
					show("Yellow Edges", yellowEdges)

					if left < 0:
							left = 0
					if right < 0:
							right = 0

					if detect_uturn(blueFit):
						if len(yellowContours) > 0:
							if (cv2.contourArea(max(yellowContours, key=cv2.contourArea)) < overrideArea or (not otherNone)):
								left = uTurnForwardSpeed * BASE_SPEED if blueLeft else uTurnBackSpeed * BASE_SPEED
								right = uTurnBackSpeed * BASE_SPEED if blueLeft else uTurnForwardSpeed * BASE_SPEED
								print("Blue U-turn detected!")
					elif detect_uturn(yellowFit):
						if len(blueContours) > 0:
							if (cv2.contourArea(max(blueContours, key=cv2.contourArea)) < overrideArea or (not otherNone)):
								left = uTurnBackSpeed * BASE_SPEED if blueLeft else uTurnForwardSpeed * BASE_SPEED
								right = uTurnForwardSpeed * BASE_SPEED if blueLeft else uTurnBackSpeed * BASE_SPEED
								print("Yellow U-turn detected")'''

					print(f"Left: {left}, Right: {right}")
					move(left, right) if DRIVER_INITIALIZED else 0
					#time.sleep(0.005)

			try:
					if cv2.waitKey(1) & 0xff == ord('q'):
							off()
							break
			except:
					off()
					pass
except:
	print("ignore this if it's not something with the code!")
	off()
	pass

off()
cap.release()
cv2.destroyAllWindows()
exit(0)