import socketserver
from utils.motor_lib.driver import move, off

class MyTCPHandler(socketserver.BaseRequestHandler):
    socketserver.TCPServer.allow_reuse_address = True
    def handle(self):
            self.data = format(self.request.recv(1024).strip().decode())
            self.request.sendall(b"HANDSHAKE OK!")

            split_data = self.data.split(" ")
            left = int(float(split_data[0]))
            right = int(float(split_data[1]))

            print(f"\nRECEIVED FROM CLIENT {format(self.client_address[0])}: {(left, right)}")

            move(left, right)

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 6969
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)

    print(f"Initialized! Listening at {HOST}:{PORT} - awaiting connections.")
    server.serve_forever()