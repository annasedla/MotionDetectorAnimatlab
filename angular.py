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
import math
from matplotlib.path import Path

# print settings
np.set_printoptions(threshold=sys.maxsize)

width,height = 64,64
xCenter = 32
yCenter = 32
radius = 32
x, y = np.mgrid[:height, :width]
coors=np.hstack((x.reshape(-1, 1), y.reshape(-1,1)))

# CONSTANTS
# Constant for pixel differences
CONSTANT = 1

# Size of the header for a message to be sent over
HEADER_SIZE = 4
DATA_SIZE = 3
FOOTER_SIZE = 1
ROBOT_TIME_STEP = 0.05

# Serial communication
ser = serial.Serial("/dev/serial0")
ser.baudrate = 115200

 
def makeMask(n, gray_scale):

    theta = np.linspace((2*math.pi)*(float(n)/64), (2*math.pi)*float(n+1)/64, 5)
    c = np.row_stack((np.column_stack((radius * np.cos(theta) + xCenter,radius * np.sin(theta) + yCenter)),
                      np.array([xCenter, yCenter])))
    
    z = Path(c).contains_points(coors).reshape(height, width)
    z_final = np.where(z == False, 0, 1)
    num_of_pixels_in_bin = np.count_nonzero(z_final == 1)
    ray = np.multiply(z_final, gray_scale)
    current_avg_at_loc = int(round(ray.sum()/num_of_pixels_in_bin))

    return current_avg_at_loc

def main():              
    # Measure how long it takes to send data over
    then = 0

    # Array to be storing the average values
    current_avg = [0] * 64

    with picamera.PiCamera() as camera:

        camera.resolution = (64, 64)
        camera.framerate = 24
        time.sleep(2)

        while True:
            start = time.time()

            # Set previous to current
            previous_avg = current_avg[:]

            # Number of elements from one image to another
            movement_length = 0

            # Checksum is 0
            checksum = 0

            # Size of data to be sent over
            size = 0

            output = np.empty((64 * 64 * 3,), dtype=np.uint8)
            camera.capture(output, 'rgb', use_video_port=True)
            output = output.reshape((64, 64, 3))

            # Correctly formatted output in RGB
            image = output[:64, :64, :]
            gray_scale = (image[:,:,0] * 0.3 + image[:,:,1] * 0.59 + image[:,:,2] * 0.11)/16

##            f = lambda x: makeMask(x, gray_scale)
##
##            current_avg = makeMask(f(current_avg))

            for n in range(0, 63):
                current_avg[n] = makeMask(n, gray_scale)

            #Count array difference between previous and current
            movement_length = ((np.array(current_avg) - np.array(previous_avg)) > CONSTANT).sum()

            if movement_length > 0:
                
                print("movement")

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
                size = HEADER_SIZE  + movement_length * DATA_SIZE + FOOTER_SIZE

                # Write the size of everything
                ser.write(bytes(bytearray([size])))

                # Add it to checksum again
                checksum += size

                for x in range (0, 63):
                    if abs(current_avg[x] - previous_avg[x]) > CONSTANT:

                        # ID
                        ser.write(bytes(bytearray([x])))

                        # low byte
                        ser.write(bytes(bytearray([current_avg[x]])))

                        # high byte
                        ser.write(bytes(bytearray([0])))

                        checksum += x + current_avg[x] + 0


                # Module checksum and send it
                checksum = checksum % 256

                # Lastly send the checksum
                ser.write(bytes(bytearray([checksum])))

                # Starting timer to when the last info was sent
                
                then = time.time()

            end = time.time()
            print (end - start)

        # Close the serial
        ser.close

if __name__ == '__main__':
    main()


