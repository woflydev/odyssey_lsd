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

            data = b'' ### CHANGED
            payload_size = struct.calcsize("L") ### CHANGED
            # Retrieve message size
            while len(data) < payload_size:
                data += conn.recv(4096)

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0] ### CHANGED

            # Retrieve all data based on message size
            while len(data) < msg_size:
                data += conn.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            # Extract frame
            frame = pickle.loads(frame_data)

            move(left, right)

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 6969
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)

    print(f"Initialized! Listening at {HOST}:{PORT} - awaiting connections.")
    server.serve_forever()