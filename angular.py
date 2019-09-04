import serial
import io
import time
import threading
import picamera
from PIL import Image
import numpy as np
import argparse
import pylab as plt
from matplotlib.path import Path
import math
import cv2

# CONSTANTS
# Constant for pixel differences
CONSTANT = 0

# width and height of an image
width, height=64, 64

# Size of the header for a message to be sent over
HEADER_SIZE = 4
DATA_SIZE = 3
FOOTER_SIZE = 1

# Number of loops
loops = 100   
 
# Serial communication
ser = serial.Serial('/dev/serial0')
ser.baudrate = 115200
                    
# Start a timer for analysis
then = 0
ROBOT_TIME_STEP = 0.1

# Array to be storing analyzed images
currentPic = []

# Array to be storing the average values
current = [0] * 64

class ImageProcessor(threading.Thread):
    def __init__(self, owner):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        self.start()

    def run(self):

        def makeMask(n):
            theta = np.linspace((2*math.pi)*(float(n)/64), (2*math.pi)*float(n+1)/64, 5)
            xCenter = 32;
            yCenter = 32;
            radius = 32;
            x = radius * np.cos(theta) + xCenter;
            y = radius * np.sin(theta) + yCenter;
            origin = np.array([xCenter, yCenter])
            c = np.column_stack((x,y))
            c = np.row_stack((c,origin))
            poly_path=Path(c)
            x, y = np.mgrid[:height, :width]
            coors=np.hstack((x.reshape(-1, 1), y.reshape(-1,1)))
            mask = poly_path.contains_points(coors)
            z = mask.reshape(height, width)
            z_final = np.where(z == False, 0, 1)
            num_of_pixels_in_bin = np.count_nonzero(z_final == 1)
            return z_final, num_of_pixels_in_bin

        try: 
            # This method runs in a separate thread
            while True:
                # Wait for an image to be written to the stream
                if self.event.wait(1):
                    try:
                        self.stream.seek(0)
                        # Read the image and do some processing on it
                        image = Image.open(self.stream).convert('L')

                        # Load it to the current array
                        currentPic = image.load()

                        # Set previous to current
                        previous = current[:]

                        finalPic = np.zeros(shape=(64,64))

                        # Convert to 4 bit
                        for x in range(0, 63):
                            for y in range(0, 63):
                                finalPic[x][y] = currentPic[x,y]/16

                        for x in range(0, 63):
                            matrix, numPixels = makeMask(x)     
                            ray = np.multiply(matrix, finalPic)
                            current[x] = int(round(ray.sum()/numPixels))

                        # Start with movement equal to false
                        movement = False

                        # Number of changed elements from one image to another
                        movementLength = 0

                        # Start with checksum equal to zero
                        checksum = 0

                        # Size of the data to be sent over
                        size = 0

                        # Run a loop for array difference
                        for x in range (0,63):
                            if abs(current[x] - previous[x]) > CONSTANT:
                                movementLength = movementLength + 1
                                movement = True

                        # Initializer
                        if movement == True:

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
                                if abs(current[x] - previous[x]) > CONSTANT:
                                    print(x)

                                    # ID
                                    ser.write(bytes(bytearray([x])))

                                    # low byte
                                    ser.write(bytes(bytearray([current[x]])))

                                    # high byte
                                    ser.write(bytes(bytearray([0])))

                                    checksum += x + current[x] + 0

                            print ("----------------")

                            # Module checksum and send it
                            checksum = checksum % 256

                            # Lastly send the checksum
                            ser.write(bytes(bytearray([checksum])))

                            # Starting timer to when the last info was sent
                            
                            then = time.time()

                        if movement != False:
                            movement = False

                    finally:
                        # Reset the stream and event
                        self.stream.seek(0)
                        self.stream.truncate()
                        self.event.clear()
                        # Return ourselves to the available pool
                        with self.owner.lock:
                            self.owner.pool.append(self)
        except KeyboardInterrupt:
            pass
            camera.stop_recording()
            ser.close

class ProcessOutput(object):
    def __init__(self):
        self.done = False
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.pool = [ImageProcessor(self) for i in range(1)]
        self.processor = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            proc = None
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True 
            proc.join()

with picamera.PiCamera(resolution='VGA') as camera:
    camera.resolution = (64,64)
    #camera.color_effects = (128,128)
    camera.start_preview()
    time.sleep(2)
    output = ProcessOutput()
    camera.start_recording(output, format='mjpeg')
    while not output.done:
        camera.wait_recording(1)
    camera.stop_recording()
    ser.close
