import socket
import sys
import random
import time

HOST, PORT = "127.0.0.1", 6969

while True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        # Connect to server and send data
        sock.connect((HOST, PORT))
        values = (random.randint(0, 100), random.randint(0, 100))
        sock.sendall(bytes(f"{values}", "utf-8"))

        # Receive data from the server and shut down
        received = str(sock.recv(1024), "utf-8")

    print("\nREQUEST:   {}".format(values))
    print("RESPONSE:  {}".format(received))

    time.sleep(0.1)