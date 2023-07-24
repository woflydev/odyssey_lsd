import socketserver
import struct
import socket
import cv2
import pickle
import threading
import socketserver
import signal
import numpy as np
from a_pipeline import pipeline
from tools import pwm

####################################################################################################

HOST = '0.0.0.0'
PWM_PORT = 6969
VIDEO_PORT = 6970

BASE_SPEED = 30
MODEL_SENSITIVITY = 0.18 # 0 to 1

####################################################################################################

left = 0
right = 0

video_data = b''
video_payload_size = struct.calcsize("L")

class TCPHandler(socketserver.BaseRequestHandler):
	socketserver.TCPServer.allow_reuse_address = True

	def handle(self):
		self.data = format(self.request.recv(1024).strip().decode())

		export = f"{left} {right}"
		
		self.request.sendall(bytes(export, "utf-8"))
		
		print(f"\nRECEIVED FROM CLIENT {format(self.client_address[0])}: {self.data}")
		print(f"SENT TO CLIENT {format(self.client_address[0])}: {export}")

def get_ip():
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.settimeout(0)
	try:
		# doesn't even have to be reachable
		s.connect(('10.254.254.254', 1))
		ip = s.getsockname()[0]
	except Exception:
			ip = '127.0.0.1'
	finally:
			s.close()
	return ip

def exit_handler(signal, frame):
	msg = "KEYBOARD INTERRUPT!"
	print(msg, end="", flush=True)
	cv2.destroyAllWindows()
	exit()

####################################################################################################

if __name__ == "__main__":
	print("HOST IP: " + get_ip())
	
	pwm_server = socketserver.TCPServer((HOST, PWM_PORT), TCPHandler)
	pwm_thread = threading.Thread(target=pwm_server.serve_forever, daemon=True).start()
	print(f"PWM SERVER INITIALIZED - AWAITING CONNECTIONS AT {get_ip()}:{PWM_PORT}")

	video_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	video_server.bind((HOST, VIDEO_PORT))
	video_server.listen(10)

	print(f"VIDEO SERVER INITIALIZED - AWAITING CONNECTIONS AT {get_ip()}:{VIDEO_PORT}")
	conn, addr = video_server.accept() # blocking function

####################################################################################################

	signal.signal(signal.SIGINT, exit_handler)

	stop = False
	angle = 90
	while True:
		if cv2.waitKey(1) & 0xFF == ord("q"):
			print("\nSERVER COMPLETED GRACEFUL SHUTDOWN!")
			cv2.destroyAllWindows()
			break
		
		while len(video_data) < video_payload_size:
			video_data += conn.recv(4096)

		packed_msg_size = video_data[:video_payload_size]
		video_data = video_data[video_payload_size:]
		msg_size = struct.unpack("L", packed_msg_size)[0]

		while len(video_data) < msg_size:
			video_data += conn.recv(4096)

		frame_data = video_data[:msg_size]
		video_data = video_data[msg_size:]
		
		frame = pickle.loads(frame_data)
		
		angle = pipeline(frame, angle, MODEL_SENSITIVITY, SHOW_IMAGES=True)
		left, right = pwm(BASE_SPEED, angle - 90)
		print(f"Motor Left: {left}, Motor Right: {right}")

		if cv2.waitKey(1) & 0xFF == 32:
			left = 0
			right = 0
			stop = not stop
		
		cv2.imshow("received", frame)
		cv2.waitKey(1) # this is required for some reason

####################################################################################################