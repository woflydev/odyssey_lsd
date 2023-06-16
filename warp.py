import sys
import cv2
import numpy as np

# Function to handle mouse events
def mouse_event(event, x, y, flags, param):
    global points, count, image_copy

    if event == cv2.EVENT_LBUTTONDOWN:
        if count < 4:
            points[count] = (x, y)
            count += 1
            print(f"Selected point {count}: ({x}, {y})")

            # Draw a circle at the selected point
            cv2.circle(image_copy, (x, y), 5, (0, 255, 0), -1)

            # Connect the points with lines
            if count > 1:
                cv2.line(image_copy, points[count-2], points[count-1], (0, 255, 0), 2)

            # If all points are selected, connect the last and first points
            if count == 4:
                cv2.line(image_copy, points[3], points[0], (0, 255, 0), 2)

            # Show the image with indicators after each point selection
            cv2.imshow('Select Points', image_copy)

# Get the image name from command-line arguments or use the default value
image_name = sys.argv[1] if len(sys.argv) > 1 else 'example1.jpg'

# Load the image
image = cv2.imread(image_name)

if image is None:
    print(f"Failed to load image: {image_name}")
    sys.exit(1)

# Create a copy of the image for drawing indicators
image_copy = image.copy()

# Initialize the points array and counter
points = [(0, 0)] * 4
count = 0

# Create a window and set the mouse callback function
cv2.namedWindow('Select Points')
cv2.setMouseCallback('Select Points', mouse_event)

# Display the initial image
cv2.imshow('Select Points', image_copy)

# Wait for points selection
while True:
    key = cv2.waitKey(1) & 0xFF

    # Break the loop if 'q' is pressed or all points are selected
    if key == ord('q') or count == 4:
        break
    elif key == ord('r'):
        # Reset the points and counter
        points = [(0, 0)] * 4
        count = 0
        # Reset the image copy
        image_copy = image.copy()
        # Clear the previous indicators and show the initial image
        cv2.destroyWindow('Select Points')
        cv2.namedWindow('Select Points')
        cv2.setMouseCallback('Select Points', mouse_event)
        cv2.imshow('Select Points', image_copy)

cv2.destroyAllWindows()

# Define the destination coordinates for the perspective warp
width, height = 400, 300
dest_points = [(0, 0), (width, 0), (width, height), (0, height)]

# Calculate the perspective transformation matrix
matrix = cv2.getPerspectiveTransform(
    src=np.float32(points),
    dst=np.float32(dest_points)
)

# Apply the perspective warp
warped_image = cv2.warpPerspective(image, matrix, (width, height))

# Display the original and warped images
cv2.imshow('Original Image', image)
cv2.imshow('Warped Image', warped_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
