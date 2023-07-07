import cv2
import numpy as np
import time
import math

VIDEO_SOURCE = 0    

SHOW_IMAGES = True
BASE_SPEED = 30

BLUR_KERNEL = 10
# for testing only [0, 62, 0], [179, 255, 124]
LOW_BLUE = [0, 0, 0]
HIGH_BLUE = [0, 0, 0]
LOW_YELLOW = [0, 62, 0]
HIGH_YELLOW = [179, 255, 124]

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

def stabilize(current, new, num_lanes, max_confident_deviation=8, max_unsure_deviation=4):
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
	if abs(angle_deviation) > max_angle_deviation:
		stabilized_steering_angle = int(current
										+ max_angle_deviation * angle_deviation / abs(angle_deviation))
	else:
		stabilized_steering_angle = new
	print('INFO: Proposed angle: %s, stabilized angle: %s' % (new, stabilized_steering_angle))
	return stabilized_steering_angle

def show(window_name, frame, show_img):
	if show_img:
		cv2.imshow(window_name, frame)

def heading(frame, angle):
	heading_image = np.zeros_like(frame)
	height, width, _ = frame.shape

	# figure out the heading line from steering angle
	# heading line (x1,y1) is always center bottom of the screen
	# (x2, y2) requires a bit of trigonometry

	radians = angle / 180.0 * math.pi
	x1 = int(width / 2)
	y1 = height
	x2 = int(x1 - height / 2 / math.tan(radians))
	y2 = int(height / 2)

	cv2.line(heading_image, (x1, y1), (x2, y2), (0, 0, 255), 10)
	heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

	return heading_image

cap = cv2.VideoCapture(VIDEO_SOURCE, cv2.CAP_DSHOW)
input("Press Enter to start analysing frames")

previousYellowAngle = None
previousBlueAngle = None
angle = 90
cutoff = 1/2
threshold = 10
while True:
    error = 0
    ret, frame = cap.read()

    if ret:
        frame = cv2.blur(frame, (BLUR_KERNEL, BLUR_KERNEL))
        contourFrame = np.copy(frame)

        low_b = np.uint8(LOW_BLUE)
        high_b = np.uint8(HIGH_BLUE)

        low_y = np.uint8(LOW_YELLOW)
        high_y = np.uint8(HIGH_YELLOW)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blueMask = cv2.inRange(hsv, low_b, high_b)
        yellowMask = cv2.inRange(hsv, low_y, high_y)

        # Only focuses on the bottom half or section of the screen as determined by cutoff
        cv2.rectangle(blueMask, (0, 0), (frame.shape[1], round(frame.shape[0] * (1 - cutoff))), 0, -1)
        cv2.rectangle(yellowMask, (0, 0), (frame.shape[1], round(frame.shape[0] * (1 - cutoff))), 0, -1)

        blueContours, hierarchy = cv2.findContours(blueMask, 1, cv2.CHAIN_APPROX_NONE) # then I used the contours method to introduce the contours in the masked image
        yellowContours, hierarchy = cv2.findContours(yellowMask, 1, cv2.CHAIN_APPROX_NONE) # then I used the contours method to introduce the contours in the masked image

        fracOffset = 1/16

        blueEndPoint = (frame.shape[1] * fracOffset, frame.shape[0])
        yellowEndPoint = ((1 - fracOffset) - frame.shape[1], frame.shape[0])

        blueAngle = None
        yellowAngle = None

        if len(blueContours) > 0:
            c_b = max(blueContours, key=cv2.contourArea)
            #print(c_b.shape)
            M = cv2.moments(c_b)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                tmpBlueEndPoint = max(np.reshape(c_b, (c_b.shape[0], c_b.shape[2])), key=lambda x: x[1])
                endPoint = blueEndPoint
                previousBlueAngle = blueAngle
                blueAngle = 180 - round(np.arctan2(endPoint[1] - cy, cx - endPoint[0]) * 180 / np.pi)
                cv2.drawContours(contourFrame, [c_b], 0, (0, 0, 255), 3)
                cv2.circle(contourFrame, (cx,cy), 5, (255,255,255), -1)
                cv2.line(contourFrame, (cx, cy), (round(endPoint[0]), endPoint[1]), (0, 255, 0), 5)     
                #print(f"Blue steering angle: {blueAngle} degrees")

        if len(yellowContours) > 0:
            c_y = max(yellowContours, key=cv2.contourArea)
            M = cv2.moments(c_y)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00']) 
                tmpYellowEndPoint = max(np.reshape(c_y, (c_y.shape[0], c_y.shape[2])), key=lambda x: x[1])
                endPoint = yellowEndPoint
                previousYellowAngle = yellowAngle
                yellowAngle = 180 - round(np.arctan2(endPoint[1] - cy, cx - endPoint[0]) * 180 / np.pi) 
                cv2.drawContours(contourFrame, [c_y], 0, (0, 0, 255), 3)
                cv2.circle(contourFrame, (cx,cy), 5, (255,255,255), -1)
                cv2.line(contourFrame, (cx, cy), (round(endPoint[0]), endPoint[1]), (0, 255, 0), 5)       
                #print(f"Yellow steering angle: {yellowAngle} degrees")

                

        if blueAngle is None and yellowAngle is not None:
            if previousYellowAngle is not None:
                angle = stabilize(yellowAngle, previousYellowAngle, 1)
            else:
                angle = yellowAngle
        elif yellowAngle is None and blueAngle is not None:
            if previousBlueAngle is not None:
                angle = stabilize(blueAngle, previousBlueAngle, 1)
            else:
                angle = yellowAngle
        elif blueAngle is not None and yellowAngle is not None:
            if previousBlueAngle is not None and previousYellowAngle is not None:
                angle = stabilize((blueAngle + yellowAngle) / 2, (previousBlueAngle + previousYellowAngle) / 2)
            else:
                angle = (blueAngle + yellowAngle) / 2
        else:
            print("give up lol")
            angle = 90
        
        heading_img = heading(contourFrame, angle)

        show("Blue Mask", blueMask, SHOW_IMAGES)
        show("Yellow Mask", yellowMask, SHOW_IMAGES)
        show("Frame", frame, SHOW_IMAGES)
        show("Contours", contourFrame, SHOW_IMAGES)
        show("Heading", heading_img, SHOW_IMAGES)

        print(f"Steering angle: {angle} degrees")

        left, right = pwm(BASE_SPEED, angle - 90)

        if left < 0:
            left = 0
        if right < 0:
            right = 0

        print(f"Left: {left}, Right: {right}")

        time.sleep(0.005)

    try:
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    except KeyboardInterrupt:
        pass

cap.release()
cv2.destroyAllWindows()
exit(0)