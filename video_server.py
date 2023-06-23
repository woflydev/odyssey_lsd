import struct
import socket
import cv2
import pickle

HOST = '0.0.0.0'
PORT = 8089

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(10)
conn, addr = s.accept()

data = b''
payload_size = struct.calcsize("L")

print('VIDEO SERVER INITIALIZED - AWAITING CONNECTIONS AT ' + HOST + ':' + str(PORT))

while True:
    # Retrieve message size
    while len(data) < payload_size:
        data += conn.recv(4096)

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += conn.recv(4096)

    frame_data = data[:msg_size]
    data = data[msg_size:]

    # Extract frame
    frame = pickle.loads(frame_data)

    # Display
    cv2.imshow('frame', frame)
    cv2.waitKey(1)