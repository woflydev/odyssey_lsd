from socketserver import BaseServer
import struct
import socket
import cv2
import pickle
import threading
import socketserver

####################################################################################################

HOST = '0.0.0.0'
PWM_PORT = 6969
VIDEO_PORT = 6970

####################################################################################################

left = 0
right = 0

class TCPHandler(socketserver.BaseRequestHandler):
	socketserver.TCPServer.allow_reuse_address = True

	def handle(self):
		self.data = format(self.request.recv(1024).strip().decode())
		self.request.sendall(b"HANDSHAKE OK!")

		split_data = self.data.split(" ")
		left = int(float(split_data[0]))
		right = int(float(split_data[1]))

		print(f"\nRECEIVED FROM CLIENT {format(self.client_address[0])}: {(self.left, self.right)}")

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

####################################################################################################

if __name__ == "__main__":
	print("HOST IP: " + get_ip())
	
	pwm_server = socketserver.TCPServer((HOST, PWM_PORT), TCPHandler)
	threading.Thread(target=pwm_server.serve_forever).start()
	print(f"PWM SERVER INITIALIZED - AWAITING CONNECTIONS AT {HOST}:{PWM_PORT}")

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, VIDEO_PORT))
	s.listen(10)
	data = b''
	payload_size = struct.calcsize("L")
	
	print(f"VIDEO SERVER INITIALIZED - AWAITING CONNECTIONS AT {HOST}:{VIDEO_PORT}")
	conn, addr = s.accept() # blocking function

####################################################################################################

	while True:
		while len(data) < payload_size:
			data += conn.recv(4096)

		packed_msg_size = data[:payload_size]
		data = data[payload_size:]
		msg_size = struct.unpack("L", packed_msg_size)[0]

		while len(data) < msg_size:
			data += conn.recv(4096)

		frame_data = data[:msg_size]
		data = data[msg_size:]
		
		frame = pickle.loads(frame_data)

		cv2.imshow('frame', frame)
		
		if cv2.waitkey(1) & 0xFF == ord('q'):
			break

	exit()

####################################################################################################