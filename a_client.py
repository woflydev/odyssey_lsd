import cv2
import socket
import pickle
import struct
import signal
import time
from utils.motor_lib.driver import move, off

####################################################################################################

SERVER_IP = 'localhost'
PWM_PORT = 6969
VIDEO_PORT = 6970

VIDEO_SOURCE = 0
CAPTURE_INTERVAL = 1 # in seconds # not used right now

####################################################################################################

def request_pwm(sock, videosocket, frame, values):
	export_frame = pickle.dumps(frame)
	message_size = struct.pack("L", len(export_frame))
	videosocket.sendall(message_size + export_frame)

	#sock.sendall(bytes(values, "utf-8"))
	sock.sendall(values.encode())
	
	try:
		received = str(sock.recv(1024), "utf-8")
	except:
		print("CONNECTION DROPPED!")
		exit()

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
cap = cv2.VideoCapture(VIDEO_SOURCE)
fps = int(round(cap.get(cv2.CAP_PROP_FPS)))
multi = fps * CAPTURE_INTERVAL
frame_id = 0
print("VIDEO SOURCE INITIALIZED! FPS: " + str(fps) + " | MULTIPLIER: " + str(multi) + " | CAPTURE INTERVAL: " + str(CAPTURE_INTERVAL) + "s")
time.sleep(0.1)

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
	frame_id += 1
	ret, frame = cap.read()
	
	frame = cv2.resize(frame, (640, 640)) # W, H
	
	try:
		pwmsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		pwmsocket.connect((SERVER_IP, PWM_PORT))
		pwm = request_pwm(pwmsocket, videosocket, frame, "req_pwm")
		pwmsocket.close()
		
		left = pwm[0]
		right = pwm[1]
		
		move(left, right)
	except:
		print("CONNECTION DROPPED!")
		off()
		exit()
	
	if ret == False:
		print("NO VIDEO FEED FOUND!")
		cap.release()
		break

	"""if frame_id % multi == 0:
		pwmsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		pwmsocket.connect((SERVER_IP, PWM_PORT))
		pwm = request_pwm(pwmsocket, videosocket, frame, "req_pwm")
		pwmsocket.close()
	else:
		print("\nFRAME SKIPPED!")"""

####################################################################################################