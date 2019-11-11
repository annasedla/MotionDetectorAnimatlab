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
##import cv2
import sys
import struct
import math
from matplotlib.path import Path
# print settings
np.set_printoptions(threshold=sys.maxsize)

# Mask settings
width,height = 64,64
xCenter = 32
yCenter = 32
radius = 32
x, y = np.mgrid[:height, :width]
coors=np.hstack((x.reshape(-1, 1), y.reshape(-1,1)))

z_final_list = []
num_of_pixels_in_bin_list = []

# CONSTANTS
# Constant for pixel differences
CONSTANT = 0

# Size of the header for a message to be sent over
HEADER_SIZE = 4
DATA_SIZE = 3
FOOTER_SIZE = 1
ROBOT_TIME_STEP = 0.05

# Measure how long it takes to send data over
then = 0

# Serial communication
ser = serial.Serial("/dev/serial0")
ser.baudrate = 115200

 
def makeMask(n):

    theta = np.linspace((2*math.pi)*(float(n)/64), (2*math.pi)*float(n+1)/64, 5)
    c = np.row_stack((np.column_stack((radius * np.cos(theta) + xCenter,radius * np.sin(theta) + yCenter)),
                      np.array([xCenter, yCenter])))
    
    z = Path(c).contains_points(coors).reshape(height, width)
    z_final = np.where(z == False, 0, 1)
    num_of_pixels_in_bin = np.count_nonzero(z_final == 1)

    return z_final, num_of_pixels_in_bin

def main():

    # Array to be storing the average values
    current_avg = [0] * 64

    # Set up blob detector
##    detector = cv2.SimpleBlobDetector()
##    params = cv2.SimpleBlobDetector_Params()
##
##    params.filterByCircularity = True

    # Populate the mask array
    for n in range(0, 64):
        z_final, num_of_pixels_in_bin = makeMask(n)
        z_final_list.append(z_final)
        num_of_pixels_in_bin_list.append(num_of_pixels_in_bin)

    # Figure out the center of the camera
##    with picamera.PiCamera() as camera:
##
##        camera.resolution = (128, 128)
##        camera.framerate = 24
##        time.sleep(2)
##        camera.capture('foo.jpg')
##
##    im = cv2.imread("foo.jpg", cv2.IMREAD_GRAYSCALE)
##    keypoints = detector.detect(im)
##
##    for keypoint in keypoints:
##        x = keypoint.pt[0]
##        y = keypoint.pt[1]
##
##    print("x: ", x, ", y: ", y )
##
##    im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
##    cv2.imshow("KEYPOINTS", im_with_keypoints)
##    cv2.waitKey(0)
##    
    with picamera.PiCamera() as camera:

        camera.resolution = (128, 128)
        camera.framerate = 24
        time.sleep(2)

        i = 0

        while True:
            start = time.time()

            # Set previous to current
            previous_avg = current_avg[:]
            current_avg = [0] * 64

            # Number of elements from one image to another
            movement_length = 0

            # Checksum is 0
            checksum = 0

            # Size of data to be sent over
            size = 0

            output = np.empty((128 * 128 * 3,), dtype=np.uint8)
            camera.capture(output, 'rgb', use_video_port=True)
            output = output.reshape((128, 128, 3))

            # Correctly formatted output in RGB
            image = output[:128, :128, :]
            gray_scale = ((image[:,:,0] * 0.3 + image[:,:,1] * 0.59 + image[:,:,2] * 0.11)/16).astype(int)
            
##            print(gray_scale)

##            current_avg = np.linspace(0,63,num=64)

##            f = lambda x : int(round((np.multiply(z_final_list[x], gray_scale)).sum()
##                                     /num_of_pixels_in_bin_list[x]))
##
##            np.apply_along_axis(f, 0, current_avg)

            # only select columns and rows from 33 to 96 (the middle portion of the image)

            row_idx = np.linspace(43,106, num = 64).astype(int)
            col_idx = np.linspace(35,98, num = 64).astype(int)
            gray_scale = gray_scale[row_idx[:, None], col_idx]

##            a = np.array([[0,1,2,3],[4,5,6,7],[8,9,10,11],[12,13,14,15]])
##
##            row_idx = np.array([1,2])
##            col_idx = np.array([2,3])
##
##            a = a[row_idx[:, None], col_idx]
##            print(a)

            for n in range(0, 64):
                ray = np.multiply(gray_scale, z_final_list[n])
                current_avg[n] = round(ray.sum()/num_of_pixels_in_bin_list[n])
##                if n == 6:
##                    print (ray)
##                print(n, ': ', current_avg[n])

            # Change this array to integer arrray
            current_avg = np.array(current_avg).astype(int)

            #Count array difference between previous and current
            for x in range (0, 64):
                if abs(current_avg[x] - previous_avg[x]) > CONSTANT:
                    movement_length = movement_length + 1;

            if movement_length > 0:
                
                now = time.time()

                global then
                time_diff = now - then

                print(time_diff)

                if time_diff != now:
                    #print(time_diff)
                    if time_diff < ROBOT_TIME_STEP:
                        time.sleep(ROBOT_TIME_STEP - time_diff)
                
                # Send the header
                ser.write(bytes([255]))
                ser.write(bytes([255]))
                ser.write(bytes([1]))
                
                # Add it to the checksum
                checksum = 255 + 255 + 1 
                
                # Compute the size of the entire message
                size = HEADER_SIZE  + movement_length * DATA_SIZE + FOOTER_SIZE

                # Write the size of everything
                ser.write(bytes([size]))

                # Add it to checksum again
                checksum += size

                for x in range (0, 64):
                    if abs(current_avg[x] - previous_avg[x]) > CONSTANT:

                        # ID
                        ser.write(bytes([x]))

                        # low byte
                        ser.write(bytes([current_avg[x]]))
##
                        # high byte
                        ser.write(bytes([0]))

                        checksum = checksum + x + current_avg[x] + 0

                # Module checksum and send it
                checksum = checksum % 256

                # Lastly send the checksum
                ser.write(bytes([checksum]))

                # Starting timer to when the last info was sent
                
                then = time.time()

            end = time.time()
            i = i + 1
        # Close the serial
        ser.close

if __name__ == '__main__':
    main()


