import cv2

cap = cv2.VideoCapture(4)

while True:
	ret, frame = cap.read()

	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('w'):
		cv2.imwrite("data/photo.test.png", frame)

	elif cv2.waitKey(1) & 0xFF == ord('q'):
		break
	