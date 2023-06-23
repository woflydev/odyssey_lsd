import cv2
import numpy as np
import socket
import pickle
import struct

SERVER_IP = '192.168.0.156'

print("INITIALIZING CAMERA...")
cap = cv2.VideoCapture(0)

try:
	clientsocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	clientsocket.connect((SERVER_IP, 8089))
	print("CONNECTION ESTABLISHED!")
except:
	print("CONNECTION DROPPED!")
	exit()

while True:
	ret, frame = cap.read()
	
	if ret == False:
		print("NO VIDEO FEED FOUND!")
		break
	
	data = pickle.dumps(frame)
	message_size = struct.pack("L", len(data)) ### CHANGED
	clientsocket.sendall(message_size + data)