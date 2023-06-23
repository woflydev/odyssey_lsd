import socket
import sys
import random
import time

HOST, PORT = "127.0.0.1", 6969

def send(host=HOST, port=PORT, values=(0, 0)):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        # Connect to server and send data
        sock.connect((host, port))
        sock.sendall(bytes(f"{values}", "utf-8"))

        # Receive data from the server and shut down
        received = str(sock.recv(1024), "utf-8")

    print("\nREQUEST:   {}".format(values))
    print("RESPONSE:  {}".format(received))

    time.sleep(0.1)