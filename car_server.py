import socketserver
from utils.motor_lib.driver import move, off

class MyTCPHandler(socketserver.BaseRequestHandler):
    socketserver.TCPServer.allow_reuse_address = True
    def handle(self):
        try:
            self.data = format(self.request.recv(1024).strip().decode())
            print(f"\nRECEIVED FROM CLIENT {format(self.client_address[0])}: {self.data}")
            self.request.sendall(b"HANDSHAKE OK!")

            move(self.data[0], self.data[1])
        except:
            self.request.sendall(b"INTERNAL SERVER ERROR!")
            print("INTERNAL SERVER ERROR!")

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 6969
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)

    print(f"Initialized! Listening at {HOST}:{PORT} - awaiting connections.")
    server.serve_forever()