import cv2
import numpy as np

# Set up the video capture
cap = cv2.VideoCapture(0)

# Define the region of interest (ROI)
roi_width = 200
roi_height = 50
roi_offset = 50

# Define the target line color in HSV
line_color = (0, 255, 0)  # Green
point_color = (0, 0, 255)  # Red

# Define lower and upper mask values for line color
lower_mask = np.array([0, 62, 0], np.uint8)
upper_mask = np.array([179, 255, 124], np.uint8)

# Define PID constants for line following
kp = 0.2  # Proportional constant
ki = 0.0  # Integral constant
kd = 0.1  # Derivative constant
prev_error = 0
integral = 0

# Function to control the robot based on line position
def control_robot(line_position, frame_width):
    global prev_error, integral  # Declare prev_error and integral as global variables

    error = line_position - frame_width / 2
    integral += error
    derivative = error - prev_error
    prev_error = error

    # Calculate the control output
    control_output = kp * error + ki * integral + kd * derivative

    # Limit the control output to the range [-1, 1]
    control_output = np.clip(control_output, -1, 1)

    # Calculate PWM values for left and right motors
    max_pwm = 100  # Maximum PWM value
    pwm_range = 0.5 * max_pwm  # Range of PWM values for motor control

    pwm_left = int(max_pwm - control_output * pwm_range)
    pwm_right = int(max_pwm + control_output * pwm_range)

    return pwm_left, pwm_right

while True:
    # Read the frame
    ret, frame = cap.read()

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the line color
    mask = cv2.inRange(hsv_frame, lower_mask, upper_mask)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)

    # Find contours in the binary image
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the contour with the largest area
        largest_contour = max(contours, key=cv2.contourArea)

        # Create a blank image for drawing the line curve and points
        line_curve = np.zeros_like(frame)
        line_points = np.zeros_like(frame)

        # Draw the line curve by fitting a polynomial to the contour points
        if len(largest_contour) > 2:
            curve_points = np.squeeze(largest_contour)
            curve_fit = np.polyfit(curve_points[:, 1], curve_points[:, 0], deg=2)
            curve_x = np.linspace(0, frame.shape[0], num=frame.shape[0])
            curve_y = np.polyval(curve_fit, curve_x)
            curve_points = np.stack((curve_y.astype(np.int32), curve_x.astype(np.int32)), axis=1)
            cv2.polylines(line_curve, [curve_points], False, line_color, thickness=10)

            # Draw points on the line
            for point in curve_points:
                cv2.circle(line_points, tuple(point), 5, point_color, -1)

        # Control the robot based on the line position
        pwm_left, pwm_right = control_robot(curve_y[-1], frame.shape[1])

        # Overlay the line curve and points on the original frame
        frame = cv2.addWeighted(frame, 1, line_curve, 0.5, 0)
        frame = cv2.addWeighted(frame, 1, line_points, 0.5, 0)

    # Show the frame
    cv2.imshow("Line Following", frame)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()

# Print the final PWM values for the left and right motors
print("PWM Left:", pwm_left)
print("PWM Right:", pwm_right)
