import cv2
import numpy as np

cap = cv2.VideoCapture(0)

roi_width = 200
roi_height = 50
roi_offset = 50

line_color = (0, 255, 0)
outline_color = (0, 0, 255)
outline_thickness = 2

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

    max_pwm = 100
    pwm_range = 0.5 * max_pwm

    pwm_left = int(max_pwm - control_output * pwm_range)
    pwm_right = int(max_pwm + control_output * pwm_range)

    return pwm_left, pwm_right

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

            curve_y = np.linspace(frame.shape[0], frame.shape[0] - roi_height, num=frame.shape[0] - roi_height)
            curve_x = np.polyval(curve_fit, curve_y)
            curve_points = np.stack((curve_x.astype(np.int32), curve_y.astype(np.int32)), axis=1)

            # Extrapolate the line beyond the contour points
            extended_y = np.linspace(frame.shape[0] - roi_height, 0, num=frame.shape[0])
            extended_x = np.polyval(curve_fit, extended_y)
            extended_points = np.stack((extended_x.astype(np.int32), extended_y.astype(np.int32)), axis=1)

            cv2.polylines(line_curve, [extended_points], False, line_color, thickness=10)

            # Draw the outline around the line
            cv2.drawContours(line_curve, [extended_points], -1, outline_color, outline_thickness)

        pwm_left, pwm_right = control_robot(curve_x[0], frame.shape[1])

        frame = cv2.addWeighted(frame, 1, line_curve, 0.5, 0)

    cv2.imshow("Line Following", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

print("PWM Left:", pwm_left)
print("PWM Right:", pwm_right)
