import time
import picamera
import numpy as np
import serial
import io
import time
import threading
import picamera
from PIL import Image
import numpy as np
import argparse
import cv2
import sys

# print settings
np.set_printoptions(threshold=sys.maxsize)

# CONSTANTS
# Constant for pixel differences
CONSTANT = 1

# Size of the header for a message to be sent over
HEADER_SIZE = 4
DATA_SIZE = 3
FOOTER_SIZE = 1
ROBOT_TIME_STEP = 0.1

# Serial communication
ser = serial.Serial("/dev/serial0")
ser.baudrate = 115200
                    
# Measure how long it takes to send data over
then = 0

# Array to be storing the average values
currentAvg = [0] * 64

i = 0

with picamera.PiCamera() as camera:

    camera.resolution = (64, 64)
    camera.framerate = 24
    time.sleep(2)

    while i<2:
        start = time.time()

        # Set previous to current
        previousAvg = currentAvg[:]

        # Movement is false
        movement = False

        # Number of elements from one image to another
        movementLength = 0

        # Checksum is 0
        checksum = 0

        # Size of data to be sent over
        size = 0

        output = np.empty((64 * 64 * 3,), dtype=np.uint8)
        camera.capture(output, 'rgb', use_video_port=True)
        output = output.reshape((64, 64, 3))

        # Correctly formatted output in RGB
        image = output[:64, :64, :]

        # Convert to Grayscale by iterating through it and rewriting to a 4 bit 2D array
        for j in range (0,63):

            sum = 0
            
            for i in range (0,63):
                gray_scale = image[i][j][0] * 0.3 + image[i][j][1] * 0.59 + image[i][j][2] * 0.11

                gray_scale = int(gray_scale/16)

                #print(gray_scale)

                # Sum the values
                sum = sum + gray_scale
 
            # Store the average in the dark array
            print(sum/64)
            currentAvg[i] = sum/64

        print('--------------------')

        # Run a loop for array difference
        for x in range (0,63):
            if abs(currentAvg[x] - previousAvg[x]) > CONSTANT:
                #print(x)
                movementLength = movementLength + 1
                movement = True

        
        # Initializer
        if movement == True:

            print('detected movement')

            now = time.time()

            global then
            time_diff = now - then

            if time_diff != now:
                #print(time_diff)
                if time_diff < ROBOT_TIME_STEP:
                    time.sleep(ROBOT_TIME_STEP - time_diff)
            
            # Send the header
            ser.write(bytes(bytearray([255])))
            ser.write(bytes(bytearray([255])))
            ser.write(bytes(bytearray([1]))) 

            # Add it to the checksum
            checksum = 255 + 255 + 1 
            
            # Compute the size of the entire message
            size = HEADER_SIZE  + movementLength * DATA_SIZE + FOOTER_SIZE

            
            # Write the size of everything
            ser.write(bytes(bytearray([size])))

            # Add it to checksum again
            checksum += size

            for x in range (0, 63):
                if abs(currentAvg[x] - previousAvg[x]) > CONSTANT:
                    #print(x)

                    # ID
                    ser.write(bytes(bytearray([x])))

                    # low byte
                    ser.write(bytes(bytearray([currentAvg[x]])))

                    # high byte
                    ser.write(bytes(bytearray([0])))

                    checksum += x + currentAvg[x] + 0

            print ("----------------")

            # Module checksum and send it
            checksum = checksum % 256

            # Lastly send the checksum
            ser.write(bytes(bytearray([checksum])))

            # Starting timer to when the last info was sent
            
            then = time.time()

        if movement != False:
            movement = False
                

##        if i == 1:
##            print output
##        i = i +1
        i = i +1
        end = time.time()
        
        #print (end - start)

    # Close the serial
    ser.close



                        



