# client.py

import socket
import turtle
import struct
import math

# SOCKET CONSTANTS
HOST = "192.168.0.1"  # The server's hostname or IP address
PORT = 43800  # The port used by the server

# DATA INTERPRETATION CONSTANTS
RECEIVED_SIZE = 1
GESTURE_RECEIVED = 0
POSITION_RECEIVED = 1

GESTURE_SIZE = 1
GESTURE_NONE = 0
GESTURE_CALIBRATE = 1
GESTURE_WRITE = 2
GESTURE_ERASE = 3

POSITION_SIZE = 4
POSITION_UPDATE = 1

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
        padding = s.recv(3) # Additional 2 bytes of padding

        # Process Gesture
        if action == GESTURE_RECEIVED:
            gesture = int.from_bytes(s.recv(GESTURE_SIZE), BYTE_ORDER)

            if gesture == GESTURE_CALIBRATE:
                # send turtle home
                print('GESTURE: CALIBRATE')
                turtle.penup()
                turtle.home()
                turtle.clearscreen()
            elif gesture == GESTURE_WRITE:
                # send turtle pen down
                print('GESTURE: WRITE')
                turtle.pendown()
            elif gesture == GESTURE_ERASE:
                print('GESTURE: ERASE')
                turtle.penup()
                turtle.home()
            elif action == GESTURE_NONE:
                # Send turtle pen up
                print('GESTURE: NONE')
                turtle.penup()
            else:
                raise Exception("Received an invalid gesture from the microcontroller.")
        
        # Process Position
        elif action == POSITION_RECEIVED:
            xPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            yPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            zPos = struct.unpack('<f', s.recv(POSITION_SIZE))[0] * SCALE_FACTOR
            position_update = int.from_bytes(s.recv(POSITION_UPDATE), BYTE_ORDER)
            a = s.recv(3)

            if position_update:
                res = math.sqrt(xPos**2 + yPos**2)
                turtle.setposition(-res, zPos)

        else:
            raise Exception("Received an invalid command from the microcontroller.")

