import cv2
import numpy as np
import socket
import pickle
import random
import struct
import signal

####################################################################################################

SERVER_IP = 'localhost'
PWM_PORT = 6969
VIDEO_PORT = 6970

####################################################################################################

def request_pwm(sock, videosocket, frame, values):
	export_frame = pickle.dumps(frame)
	message_size = struct.pack("L", len(export_frame))
	videosocket.sendall(message_size + export_frame)

	sock.sendall(bytes(values, "utf-8"))

	received = str(sock.recv(1024), "utf-8")

	print(f"\nREQUEST:   {format(values)}", )
	print(f"RESPONSE:  {format(received)}")

	return received
	
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
	pwmsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	videosocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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

	pwmsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	pwmsocket.connect((SERVER_IP, PWM_PORT))
	#videosocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	#videosocket.connect((SERVER_IP, VIDEO_PORT))
	pwm = request_pwm(pwmsocket, videosocket, frame, "req_pwm")
	pwmsocket.close()
	#videosocket.close()

####################################################################################################