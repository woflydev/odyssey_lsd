import socketserver
import struct
import socket
import cv2
import pickle
import threading
import socketserver
import signal

####################################################################################################

HOST = '0.0.0.0'
PWM_PORT = 6969
VIDEO_PORT = 6970

####################################################################################################

left = 0
right = 0

video_data = b''
video_payload_size = struct.calcsize("L")

class TCPHandler(socketserver.BaseRequestHandler):
	socketserver.TCPServer.allow_reuse_address = True

	def handle(self):
		self.data = format(self.request.recv(1024).strip().decode())
		
		if self.data == "req_pwm":
			self.request.sendall(b"HANDSHAKE OK!")

		print(f"\nRECEIVED FROM CLIENT {format(self.client_address[0])}: {self.data}")

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

def exit_handler(signum, frame):
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

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, VIDEO_PORT))
	s.listen(10)
	
	print(f"VIDEO SERVER INITIALIZED - AWAITING CONNECTIONS AT {get_ip()}:{VIDEO_PORT}")
	conn, addr = s.accept() # blocking function

####################################################################################################

	signal.signal(signal.SIGINT, exit_handler)

	while True:
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

		cv2.imshow('frame', frame)
		cv2.waitKey(1) # this is required for some reason

####################################################################################################