import cv2
import numpy as np
import socket
import pickle
import struct

SERVER_IP = '192.168.0.156'

cap = cv2.VideoCapture(0)
clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect((SERVER_IP, 8089))

while True:
	ret,frame=cap.read()
	# Serialize frame
	data = pickle.dumps(frame)

	# Send message length first
	message_size = struct.pack("L", len(data)) ### CHANGED

	# Then data
	clientsocket.sendall(message_size + data)

print("Server Closed!")