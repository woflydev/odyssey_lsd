import cv2
import numpy as np

cap = cv2.VideoCapture(0)

line_color = (0, 255, 0)
outline_color = (0, 0, 255)
outline_thickness = 2
rectangle_color = (255, 0, 0)
rectangle_thickness = 2

lower_mask = np.array([0, 62, 0], np.uint8)
upper_mask = np.array([179, 255, 124], np.uint8)

kp = 0.2
ki = 0.0
kd = 0.1
prev_error = 0
integral = 0

def control_robot(line_position, frame_width):
    global prev_error, integral

    error = line_position - frame_width / 2
    integral += error
    derivative = error - prev_error
    prev_error = error

    control_output = kp * error + ki * integral + kd * derivative
    control_output = np.clip(control_output, -1, 1)

    steering_angle = 90 + control_output * 45  # Assuming a maximum steering angle of 45 degrees

    return steering_angle

while True:
    ret, frame = cap.read()

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_frame, lower_mask, upper_mask)

    blurred = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        line_curve = np.zeros_like(frame)

        if len(largest_contour) > 2:
            curve_points = np.squeeze(largest_contour)
            curve_fit = np.polyfit(curve_points[:, 1], curve_points[:, 0], deg=2)

            curve_y = np.linspace(frame.shape[0], 0, num=frame.shape[0])
            curve_x = np.polyval(curve_fit, curve_y)
            curve_points = np.stack((curve_x.astype(np.int32), curve_y.astype(np.int32)), axis=1)

            # Draw the line
            cv2.polylines(line_curve, [curve_points], False, line_color, thickness=2)

            # Get the bounding rectangle around the line
            rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(curve_points)

            # Draw the rectangle
            cv2.rectangle(line_curve, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), rectangle_color, rectangle_thickness)

            steering_angle = control_robot(rect_x + rect_w / 2, frame.shape[1])

            frame = cv2.addWeighted(frame, 1, line_curve, 0.5, 0)

            print(steering_angle)

    cv2.imshow("Line Following", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
