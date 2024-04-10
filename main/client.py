# client.py

import socket
import turtle
import struct
import time

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
    turtle.penup()

    # Main Loop
    while(True):
        action = int.from_bytes(s.recv(RECEIVED_SIZE), BYTE_ORDER)
        padding = s.recv(3) # Floats have additional 3 bytes of padding

        # Process Gesture
        if action == GESTURE_RECEIVED:
            gesture = int.from_bytes(s.recv(GESTURE_SIZE), BYTE_ORDER)

            if gesture == GESTURE_CALIBRATE:
                # send turtle home
                print('GESTURE: CALIBRATE')
                turtle.penup()
                turtle.home()
            elif gesture == GESTURE_WRITE:
                # send turtle pen down
                print('GESTURE: WRITE')
                turtle.pendown()
            elif gesture == GESTURE_ERASE:
                print('GESTURE: ERASE')
                turtle.penup()
                turtle.home()
                turtle.clearscreen()
            else:
                print('GESTURE INVALID')
        
        # Process Position
        elif action == POSITION_RECEIVED:
            xPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            yPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            zPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR

            turtle.setposition(xPos, yPos)

        else:
            print('TYPE INVALID')

