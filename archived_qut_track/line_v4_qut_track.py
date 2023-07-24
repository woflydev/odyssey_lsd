import cv2
import numpy as np
import time
import math

DRIVER_INITIALIZED = False
try:
	from utils.motor_lib.driver import move, off
	DRIVER_INITIALIZED = True
except:
	print("MOTOR DRIVER NOT INITIALIZED! RUNNING ANYWAY...")

VIDEO_SOURCE = 0

SHOW_IMAGES = False
WRITE_IMAGES = True
BASE_SPEED = 40
BOOST_SPEED = 50
BOOST_ANGLE = 5

BLUR_KERNEL = 10
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

LOW_YELLOW = [0, 66, 187] # qut track
HIGH_YELLOW = [46, 255, 255]

LOW_PURPLE = [117, 139, 27]
HIGH_PURPLE = [156, 255, 134]

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

def stabilize(current, new, num_lanes, max_confident_deviation=3,max_unsure_deviation=1.5):
	"""
	Using last steering angle to stabilize the steering angle
	This can be improved to use last N angles, etc
	if new angle is too different from current angle, only turn by max_angle_deviation degrees
	"""

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
	#print('INFO: Proposed angle: %s, stabilized angle: %s' % (new, stabilized_steering_angle))
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

def show(window_name, frame):
	if SHOW_IMAGES:
		cv2.imshow(window_name, frame)
	if WRITE_IMAGES:
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

cap = cv2.VideoCapture(VIDEO_SOURCE)
input("Press Enter to start analysing frames")

previousYellowAngle = None
previousBlueAngle = None
blueLeft = True
angle = 90
cutoffConstant = 1/2
fracOffset = 1/16
horizontalSlopeThreshold = 0.15
horizontalLengthThreshold = 125
cannyMin = 200
cannyMax = 400
uTurnForwardSpeed = 1.3
uTurnBackSpeed = -0.25
otherNone = False
overrideArea = 20

obstacleThreshold = 200
obstacleCorrection = 0
obstacleCompensation = 0
obstacleScale = 0.5
obstacleTurnThreshold = 30
defaultTurnRight = True
obstacleBounds = 200
obstaclePassed = False
obstacleCorrectionFrames = 0

finalAngleOffset = 0

contourColor = (0, 0, 255)
obstacleColor = (255, 0, 0)
lineColor = (0, 255, 0)
circleColor = (255, 255, 255)
circleRadius = 5
lineThickness = 3

blueAngle = None
yellowAngle = None

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

				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
				obstacleHSV = cv2.cvtColor(obstacleFrame, cv2.COLOR_BGR2HSV)

				blueMask = cv2.inRange(hsv, low_b, high_b)
				yellowMask = cv2.inRange(hsv, low_y, high_y)
				obstacleMask = cv2.inRange(obstacleHSV, low_p, high_p)

				# Only focuses on the bottom half or section of the screen as determined by cutoff
				bottomLeft = np.zeros_like(blueMask)
				bottomRight = np.zeros_like(yellowMask)

				cutoff = clamp(cutoffConstant * BASE_SPEED / 30, (1/3, 5/6))

				bottomLeft = cv2.rectangle(bottomLeft, (0, bottomLeft.shape[0]), (round(bottomLeft.shape[1] * cutoff), round(bottomLeft.shape[0] * (1 - cutoff))), 255, -1)
				bottomRight = cv2.rectangle(bottomRight, (bottomRight.shape[1], bottomRight.shape[0]), (round(bottomRight.shape[1] * (1 - cutoff)), round(bottomRight.shape[0] * (1 - cutoff))), 255, -1)

				blueMask = cv2.bitwise_and(blueMask, blueMask, mask=bottomLeft if blueLeft else bottomRight)
				yellowMask = cv2.bitwise_and(yellowMask, yellowMask, mask=bottomRight if blueLeft else bottomLeft)

				blueContours, hierarchy = cv2.findContours(blueMask, 1, cv2.CHAIN_APPROX_NONE) 
				yellowContours, hierarchy = cv2.findContours(yellowMask, 1, cv2.CHAIN_APPROX_NONE)
				# Used simple here to save memory
				obstacleContours, hierarchy = cv2.findContours(obstacleMask, 1, cv2.CHAIN_APPROX_SIMPLE)

				leftEndPoint = (frame.shape[1] * fracOffset, frame.shape[0])
				rightEndPoint = ((1 - fracOffset) * frame.shape[1], frame.shape[0])

				if len(blueContours) > 0:
						c_b = max(blueContours, key=cv2.contourArea)
						#print(c_b.shape)
						M = cv2.moments(c_b)
						if M["m00"] != 0:
								cx = int(M['m10']/M['m00'])
								cy = int(M['m01']/M['m00'])
								endPoint = leftEndPoint if blueLeft else rightEndPoint
								previousBlueAngle = blueAngle
								blueAngle = 180 - round(np.arctan2(endPoint[1] - cy, cx - endPoint[0]) * 180 / np.pi)
								cv2.drawContours(contourFrame, [c_b], 0, contourColor, lineThickness)
								cv2.circle(contourFrame, (cx,cy), circleRadius, circleColor, -1)
								cv2.line(contourFrame, (cx, cy), (round(endPoint[0]), endPoint[1]), lineColor, lineThickness)     
								#print(f"Blue steering angle: {blueAngle} degrees")
				else:
						blueAngle = None

				if len(yellowContours) > 0:
						c_y = max(yellowContours, key=cv2.contourArea)
						M = cv2.moments(c_y)
						if M["m00"] != 0:
								cx = int(M['m10']/M['m00'])
								cy = int(M['m01']/M['m00']) 
								endPoint = rightEndPoint if blueLeft else leftEndPoint
								previousYellowAngle = yellowAngle
								yellowAngle = 180 - round(np.arctan2(endPoint[1] - cy, cx - endPoint[0]) * 180 / np.pi) 
								cv2.drawContours(contourFrame, [c_y], 0, contourColor, lineThickness)
								cv2.circle(contourFrame, (cx,cy), circleRadius, circleColor, -1)
								cv2.line(contourFrame, (cx, cy), (round(endPoint[0]), endPoint[1]), lineColor, lineThickness)         
								#print(f"Yellow steering angle: {yellowAngle} degrees")
				else:
					yellowAngle = None


				'''obstacleObj = []
				if len(obstacleContours) > 0:
						obstacles = list(filter(lambda c: cv2.contourArea(c) > obstacleThreshold, obstacleContours))
						cv2.drawContours(contourFrame, obstacles, -1, obstacleColor, lineThickness)
						for obj in obstacles:
								M = cv2.moments(obj)
								if M["m00"] != 0:
										cx = int(M['m10']/M['m00'])
										cy = int(M['m01']/M['m00'])
										obstacleObj.append({"centre": [cx, cy], "contour": obj, "size": cv2.contourArea(obj)})
										cv2.circle(contourFrame, (cx,cy), circleRadius, circleColor, -1)  
						obstaclePassed = False
						obstacleCorrectionFrames += 1

						# Sorts them based on x-coordinate
						obstacleObj = sorted(obstacleObj, key=lambda obstacle: obstacle["centre"][0])
						differenceArr = []
						
						if len(obstacleObj) > 1:
								for i in range(len(obstacleObj) - 1):
										differenceArr.append(obstacleObj[i+1]["centre"][0] - obstacleObj[i]["centre"][0])
								maxDifference = 0
								maxIndex = 0
								for i in range(len(differenceArr)):
										if differenceArr[i] > maxDifference:
												maxDifference = differenceArr[i]
												maxIndex = i

								weights = (obstacleObj[maxIndex + 1]["size"], obstacleObj[maxIndex]["size"])
								
								# Taking a weighted average of the two adjacent obstacles
								point = (weighted_avg([obstacleObj[maxIndex]["centre"][0], obstacleObj[maxIndex + 1]["centre"][0]], weights),
																		weighted_avg([obstacleObj[maxIndex]["centre"][1], obstacleObj[maxIndex + 1]["centre"][1]], weights))
								obstacleCorrection = 180 - round(np.arctan2(point[1], point[0] - frame.shape[1] / 2) * 180 / np.pi)
								finalAngleOffset = obstacleCorrection
						elif len(obstacleObj) == 1:
								offset = 0
								difference = frame.shape[1] / 2 - obstacleObj[0]["centre"][0]
								if abs(difference) > obstacleTurnThreshold:
										# Arbitrary such that if the obstacle is far to the right, the car will turn slightly to the left and vice versa. The offset is proportional to how close the obstacle is (obstacleSize) and a variable
										offset = -clamp(obstacleScale * obstacleObj[0]["size"] / difference, [-obstacleBounds, obstacleBounds])
								else:
										offset = clamp((1 if defaultTurnRight else -1) * obstacleScale * obstacleObj[0]["size"], [-obstacleBounds, obstacleBounds])
								point = (frame.shape[1] / 2 + offset, frame.shape[0] / 2)
								obstacleCorrection = 180 - round(np.arctan2(point[1], point[0] - frame.shape[1] / 2) * 180 / np.pi)
								finalAngleOffset = obstacleCorrection
						else:
							finalAngleOffset = 0
				else:
						obstaclePassed = True
						if obstacleCorrectionFrames > 0:
								obstacleCompensation = -obstacleCorrection
								obstacleCorrectionFrames -= 1
						else:
								obstacleCompensation = 0
						finalAngleOffset = obstacleCompensation'''
				
				print(f"Blue angle: {blueAngle}, Yellow angle: {yellowAngle}")

				if blueAngle is None and yellowAngle is not None:
						if previousYellowAngle is not None:
								angle = stabilize(yellowAngle, previousYellowAngle, 1)
								#print("Stabilising")
						else:
								angle = yellowAngle
								#print("Not stabilising")
						previousYellowAngle = yellowAngle
				elif yellowAngle is None and blueAngle is not None:
						if previousBlueAngle is not None:
								angle = stabilize(blueAngle, previousBlueAngle, 1)
								#print("Stabilising")
						else:
								angle = blueAngle
								#print("Not stabilising")
						previousBlueAngle = blueAngle
				elif blueAngle is not None and yellowAngle is not None:
						if previousBlueAngle is not None and previousYellowAngle is not None:
								angle = stabilize((blueAngle + yellowAngle) / 2, (previousBlueAngle + previousYellowAngle) / 2, 2)
								#print("Stabilising")
						else:
								angle = (blueAngle + yellowAngle) / 2
								#print("Not stabilising")
						previousYellowAngle = yellowAngle
						previousBlueAngle = blueAngle
				else:
						print("give up lol")
						angle = 90
				#print(f"Previous angles: {previousBlueAngle}, {previousYellowAngle}")
				# Adds obstacle avoidance
				#angle += finalAngleOffset

				

				
				heading_img = heading(contourFrame, angle)

				show("Blue Mask", blueMask)
				show("Yellow Mask", yellowMask)
				show("Obstacle Mask", obstacleMask)
				show("Frame", frame)
				show("Contours", contourFrame)
				show("Heading", heading_img)

				print(f"Steering angle: {angle} degrees")

				left, right = pwm(BASE_SPEED if abs(angle - 90) > BOOST_ANGLE else BOOST_SPEED, angle - 90)

				# U-turn code
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
							print("Yellow U-turn detected")

				print(f"Left: {left}, Right: {right}")
				move(left, right) if DRIVER_INITIALIZED else 0
				time.sleep(0.005)

		try:
				if cv2.waitKey(1) & 0xff == ord('q'):
						break
		except KeyboardInterrupt:
				pass

cap.release()
cv2.destroyAllWindows()
exit(0)