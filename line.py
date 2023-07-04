import cv2
import numpy as np

# Define the desired screen resolution
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

# Define the region of interest (ROI) for line detection
ROI_TOP = 300
ROI_BOTTOM = 480

# Initialize the video capture
cap = cv2.VideoCapture(0)
cap.set(3, SCREEN_WIDTH)
cap.set(4, SCREEN_HEIGHT)

# Function to preprocess the frame for line detection
def preprocess_frame(frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Apply ROI mask
    mask = np.zeros_like(edges)
    mask[ROI_TOP:ROI_BOTTOM, :] = 255
    masked_edges = cv2.bitwise_and(edges, mask)

    return masked_edges

# Function to detect lines in the frame and calculate steering angle
def detect_lines(frame):
    lines = cv2.HoughLinesP(frame, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=100)

    if lines is not None:
        # Calculate average slope and intercept of the detected lines
        slopes = []
        intercepts = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            intercept = y1 - slope * x1
            slopes.append(slope)
            intercepts.append(intercept)

        avg_slope = np.mean(slopes)
        avg_intercept = np.mean(intercepts)

        # Calculate steering angle based on the slope
        steering_angle = np.arctan(avg_slope) * 180 / np.pi

        # Adjust the steering angle for going straight
        steering_angle += 90

        return steering_angle

    return None

# Main loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Preprocess frame for line detection
    processed_frame = preprocess_frame(frame)

    # Detect lines and calculate steering angle
    steering_angle = detect_lines(processed_frame)

    # Display the frame with line detection
    cv2.imshow('Line Following', processed_frame)

    # Draw heading visualization
    if steering_angle is not None:
        # Convert steering angle to radians
        angle_rad = np.deg2rad(steering_angle)

        # Calculate line coordinates for visualization
        center_x = SCREEN_WIDTH // 2
        center_y = ROI_BOTTOM
        line_length = 100
        line_x = int(center_x + line_length * np.sin(angle_rad))
        line_y = int(center_y - line_length * np.cos(angle_rad))

        # Draw heading line
        cv2.line(frame, (center_x, center_y), (line_x, line_y), (0, 0, 255), 2)

        # Display the frame with heading visualization
        cv2.imshow('Heading Visualization', frame)

    # Output the steering angle
    if steering_angle is not None:
        print("Steering Angle:", steering_angle)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
