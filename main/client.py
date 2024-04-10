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
GESTURE_WRITE = 1
GESTURE_ERASE = 2

POSITION_SIZE = 4

BYTE_ORDER = 'little'
SCALE_FACTOR = 500

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # Initialize
    s.connect((HOST, PORT))
    turtle.home()
    ##### CHANGE TO penup() AFTER DEBUGGING AND GESTURE CODE
    turtle.pendown()

    # Main Loop
    while(True):
        action = int.from_bytes(s.recv(RECEIVED_SIZE), BYTE_ORDER)
        a = s.recv(3)
        print(action)

        # Process Gesture
        if action == GESTURE_RECEIVED:
            print('gesture')
            gesture = int.from_bytes(s.recv(GESTURE_SIZE), BYTE_ORDER)

            if gesture == GESTURE_CALIBRATE:
                # send turtle home
                print('Home')
                turtle.penup()
                turtle.home()
            if gesture == GESTURE_WRITE:
                # send turtle pen down
                print('pen down')
                turtle.pendown()
            if gesture == GESTURE_ERASE:
                print('erase')
                turtle.penup()
                turtle.clearscreen()
        
        # Process Position
        elif action == POSITION_RECEIVED:
            print('Position')
            xPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            yPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            zPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            print(f'X: {xPos}')
            print(f'Y: {yPos}')
            print(f'Z: {zPos}')

            turtle.setposition(xPos, yPos)

