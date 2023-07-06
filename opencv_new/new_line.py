import cv2
import numpy as np
import time

SHOW_IMAGES = True
BASE_SPEED = 30

BLUR_KERNEL = 15
LOW_BLUE = [84, 0, 223]
HIGH_BLUE = [136, 255, 255]
LOW_YELLOW = [0, 0, 239]
HIGH_YELLOW = [43, 63, 255]

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
			print('Unable to calculate PWM! (Most commonly from division by zero)')

def show(window_name, frame, show_img):
	if show_img:
		cv2.imshow(window_name, frame)

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

angle = 0
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

        blueContours, hierarchy = cv2.findContours(blueMask, 1, cv2.CHAIN_APPROX_NONE) # then I used the contours method to introduce the contours in the masked image
        yellowContours, hierarchy = cv2.findContours(yellowMask, 1, cv2.CHAIN_APPROX_NONE) # then I used the contours method to introduce the contours in the masked image

        blueEndPoint = (frame.shape[1] / 8, frame.shape[0])
        yellowEndPoint = (frame.shape[1] * 7 / 8, frame.shape[0])

        blueAngle = None
        yellowAngle = None

        if len(blueContours) > 0:
            c_b = max(blueContours, key=cv2.contourArea)
            #print(c_b.shape)
            M = cv2.moments(c_b)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #blueEndPoint = max(np.reshape(c_b, (c_b.shape[0], c_b.shape[2])), key=lambda x: x[1])   
                blueAngle = 180 - round(np.arctan2(blueEndPoint[1] - cy, cx - blueEndPoint[0]) * 180 / np.pi)        
                print(f"Blue steering angle: {blueAngle} degrees")

                cv2.drawContours(contourFrame, [c_b], 0, (0, 0, 255), 3)
                cv2.circle(contourFrame, (cx,cy), 5, (255,255,255), -1)
                cv2.line(contourFrame, (cx, cy), (round(blueEndPoint[0]), blueEndPoint[1]), (0, 255, 0), 5)

        if len(yellowContours) > 0:
            c_y = max(yellowContours, key=cv2.contourArea)
            M = cv2.moments(c_y)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00']) 
                #yellowEndPoint = max(c_y, key=lambda x: x[1])   
                yellowAngle = 180 - round(np.arctan2(yellowEndPoint[1] - cy, cx - yellowEndPoint[0]) * 180 / np.pi)        
                print(f"Yellow steering angle: {yellowAngle} degrees")

                cv2.drawContours(contourFrame, [c_y], 0, (0, 0, 255), 3)
                cv2.circle(contourFrame, (cx,cy), 5, (255,255,255), -1)
                cv2.line(contourFrame, (cx, cy), (round(yellowEndPoint[0]), yellowEndPoint[1]), (0, 255, 0), 5)
        
        
        if blueAngle is None and yellowAngle is not None:
            angle = yellowAngle
        elif yellowAngle is None and blueAngle is not None:
            angle = blueAngle
        elif blueAngle is not None and yellowAngle is not None:
            angle = (blueAngle + yellowAngle) / 2
        else:
            print("give up lol")

        show("Blue Mask", blueMask, SHOW_IMAGES)
        show("Yellow Mask", yellowMask, SHOW_IMAGES)
        show("Frame", frame, SHOW_IMAGES)
        show("Contours", contourFrame, SHOW_IMAGES)

        print(f"Steering angle: {angle} degrees")

        left, right = pwm(BASE_SPEED, angle)

        print(f"Left: {left}, Right: {right}")

        time.sleep(0.005)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()