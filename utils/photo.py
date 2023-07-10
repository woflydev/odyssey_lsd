import cv2
import time

cap = cv2.VideoCapture(0)

while True:
	ret, frame = cap.read()

	time.sleep(2)
	cv2.imwrite("data/photo.test.png", frame)

	if cv2.waitKey(1) & 0xFF == ord('w'):
		cv2.imwrite("data/photo.test.png", frame)

	elif cv2.waitKey(1) & 0xFF == ord('q'):
		break