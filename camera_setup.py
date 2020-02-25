from picamera import PiCamera
from time import sleep
from PIL import Image
import numpy as np
import argparse
import cv2
import time
from io import BytesIO
from PIL import Image

# Array to be storing analyzed images
currentPic = []

# Save to an array directly
camera = PiCamera()

# minimum allowed resolution
camera.resolution = (64,64)

# start preview
camera.start_preview()

# warmp up time
time.sleep(2)

# Make camera preview B & W
camera.color_effects = (128,128)

