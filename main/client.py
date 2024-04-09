# client.py

import socket
import turtle
import struct

# SOCKET CONSTANTS
HOST = "192.168.0.1"  # The server's hostname or IP address
PORT = 43800  # The port used by the server

# DATA INTERPRETATION CONSTANTS
RECEIVED_SIZE = 1
GESTURE_RECEIVED = 0
POSITION_RECEIVED = 1

GESTURE_SIZE = 1
GESTURE_CALIBRATE = 0
GESTURE_PEN_DOWN = 1
GESTURE_PEN_UP = 2
GESTURE_ERASE = 3

POSITION_SIZE = 4

BYTE_ORDER = 'big'
SCALE_FACTOR = 1

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # Initialize
    s.connect((HOST, PORT))

    # Main Loop
    while(True):
        action = int.from_bytes(s.recv(RECEIVED_SIZE), BYTE_ORDER)
        print(action)

        # Process Gesture
        if action == GESTURE_RECEIVED:
            print('gesture')
            gesture = int.from_bytes(s.recv(GESTURE_SIZE), BYTE_ORDER)

            if gesture == GESTURE_CALIBRATE:
                # send turtle home
                print('Home')
                #turtle.home()
            if gesture == GESTURE_PEN_DOWN:
                # send turtle pen down
                print('pen down')
                #turtle.pendown()
            if gesture == GESTURE_PEN_UP:
                # send turtle pen up
                print('pen up')
                #turtle.penup()
            if gesture == GESTURE_ERASE:
                print('erase')
                #turtle.clearscreen()
        
        # Process Position
        elif action == POSITION_RECEIVED:
            xPos = struct.unpack('f', s.recv(POSITION_SIZE)) * SCALE_FACTOR
            yPos = struct.unpack('f', s.recv(POSITION_SIZE)) * SCALE_FACTOR
            zPos = struct.unpack('f', s.recv(POSITION_SIZE)) * SCALE_FACTOR
            print(xPos)
            print(yPos)
            print(zPos)

            #turtle.setposition(xPos, yPos)

            # move turtle to position
            # TODO: Scaling

