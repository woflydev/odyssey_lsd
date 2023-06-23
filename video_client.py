import cv2
import numpy as np
import socket
import pickle
import struct
import signal

####################################################################################################

SERVER_IP = 'localhost'
PWM_PORT = 6969
VIDEO_PORT = 6970

####################################################################################################

def request_pwm(s, values):
	values = b"req_pwm"
	s.sendall(bytes(f"{values}", "utf-8"))
	received = format(str(s.recv(1024), "utf-8"))

	print("\nREQUEST:   {}".format(values))
	print("RESPONSE:  {}", received)

	return received

def send_video(s, frame):
	data = pickle.dumps(frame)
	message_size = struct.pack("L", len(data))
	s.sendall(message_size + data)
	
def exit_handler(signum, frame):
	msg = "KEYBOARD INTERRUPT!"
	print(msg, end="", flush=True)
	cv2.destroyAllWindows()
	exit()

####################################################################################################

signal.signal(signal.SIGINT, exit_handler)

print("INITIALIZING CAMERA...")
cap = cv2.VideoCapture(0)

try:
	pwmsocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	videosocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	print("CONNECTING TO PWM SERVER AT " + SERVER_IP + ":" + str(PWM_PORT) + "...")
	pwmsocket.connect((SERVER_IP, PWM_PORT))
	print("CONNECTING TO VIDEO SERVER AT " + SERVER_IP + ":" + str(VIDEO_PORT) + "...")
	videosocket.connect((SERVER_IP, VIDEO_PORT))
	print("CONNECTION ESTABLISHED!")
except:
	print("CONNECTION DROPPED!")
	exit()

while True:
	ret, frame = cap.read()
	
	if ret == False:
		print("NO VIDEO FEED FOUND!")
		break
	
	send_video(videosocket, frame)

####################################################################################################