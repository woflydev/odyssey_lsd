import cv2
import numpy as np
import socket
import pickle
import random
import struct
import signal
import time

####################################################################################################

SERVER_IP = 'localhost'
PWM_PORT = 6969

####################################################################################################

def request_pwm(sock, values):

	sock.sendall(bytes(values, "utf-8"))

	received = sock.recv(1024).decode()

	print(f"\nREQUEST:   {format(values)}", )
	print(f"RESPONSE:  {format(received)}")
	
	return received
	
def exit_handler(signum, frame):
	msg = "KEYBOARD INTERRUPT!"
	print(msg, end="", flush=True)
	cv2.destroyAllWindows()
	exit()

####################################################################################################

try:
	pwmsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print("CONNECTING TO PWM SERVER AT " + SERVER_IP + ":" + str(PWM_PORT) + "...")
	pwmsocket.connect((SERVER_IP, PWM_PORT))
	print("CONNECTION ESTABLISHED!")
except:
	print("CONNECTION DROPPED!")
	exit()

while True:
	pwm = request_pwm(pwmsocket, "req_pwm")
	print(f"RECEIVED PWM: {format(pwm)}")

####################################################################################################