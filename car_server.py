import socketserver
import random

class MyTCPHandler(socketserver.BaseRequestHandler):
	socketserver.TCPServer.allow_reuse_address = True
	def handle(self):
		self.data = format(self.request.recv(1024).strip().decode())

		print(f"\nRECEIVED FROM CLIENT {format(self.client_address[0])}: {self.data}")
		
		fake_data = f"{random.randint(0, 100)} {random.randint(0, 100)}"
		self.request.sendall(bytes(fake_data, "utf-8"))

if __name__ == "__main__":
	HOST, PORT = "0.0.0.0", 6969
	server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)

	print(f"Initialized! Listening at {HOST}:{PORT} - awaiting connections.")
	server.serve_forever()