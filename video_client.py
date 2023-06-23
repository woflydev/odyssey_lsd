import cv2
import numpy as np
import socket
import pickle
import struct
import time

####################################################################################################

SERVER_IP = '192.168.0.156'
PORT = 6969

####################################################################################################

def send(server_ip=SERVER_IP, port=PORT):
	values = "req_pwm"
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
		sock.connect((server_ip, port))
		sock.sendall(bytes(f"{values}", "utf-8"))
		received = str(sock.recv(1024), "utf-8")

	print("\nREQUEST:   {}".format(values))
	print("RESPONSE:  {}".format(received))

	time.sleep(0.003)

####################################################################################################

print("INITIALIZING CAMERA...")
cap = cv2.VideoCapture(0)

try:
	print("ATTEMPTING TO CONNECT TO SERVER AT " + SERVER_IP + ":" + str(PORT) + "...")
	clientsocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	clientsocket.connect((SERVER_IP, PORT))
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
	message_size = struct.pack("L", len(data))
	clientsocket.sendall(message_size + data)
	
####################################################################################################