#!/usr/bin/env python3

import socket
import struct
import rospy
from std_msgs.msg import Float64, Bool

#Ros
rospy.init_node("StartStop")
StartStop_pub = rospy.Publisher("Start/Stop",Float64,queue_size = 10)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.43.5', 8888))  # Replace with the desired server IP and port (Raspberry pi)
def receiveData():
    server_socket.listen(1)

    print("Server listening for connections...")

    client_socket, address = server_socket.accept()
    print("Connected to:", address)

    received_data = client_socket.recv(4)  # Assuming the integer value is 4 bytes long
    value_received = struct.unpack('i', received_data)[0]

    print("Received value:", value_received)
    
    StartStop_pub.publish(value_received)
        

    #client_socket.close()
    #server_socket.close()

while True:
    receiveData()