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
import sys
import struct
import math
from matplotlib.path import Path
from PIL import Image
import cv2
    
# Figure out the center of the camera
with picamera.PiCamera() as camera:

    camera.resolution = (128, 128)
    camera.framerate = 24
    time.sleep(2)
    camera.capture('foo.jpg')

# Set up blob detector
detector = cv2.SimpleBlobDetector()
params = cv2.SimpleBlobDetector_Params()

params.filterByCircularity = True

im = cv2.imread("foo.jpg", cv2.IMREAD_GRAYSCALE)
keypoints = detector.detect(im)

for keypoint in keypoints:
    x = keypoint.pt[0]
    y = keypoint.pt[1]

print('x: ', x, ', y: ', y )

im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow("KEYPOINTS", im_with_keypoints)
cv2.waitKey(0)
