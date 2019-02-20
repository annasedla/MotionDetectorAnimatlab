import serial
import io
import time
import threading
import picamera
from PIL import Image
import numpy as np
import argparse
import cv2

# CONSTANTS
# Constant for pixel differences
CONSTANT = 1

# Size of the header for a message to be sent over
HEADER_SIZE = 3
DATA_SIZE = 3
FOOTER_SIZE = 1

# Number of loops
loops = 10

# Serial communication
ser = serial.Serial('/dev/serial0')
ser.baudrate = 19200
#ser.open()
                    
# Start a timer for analysis
start = time.time()

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
        # This method runs in a separate thread
        for i in range(0,loops):
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

                    # Convert to 4 bit
                    for y in range(0, 64):

                        # Darkest spot in the view
                        sum = 0

                        for x in range(0, 64):
                            currentPic[x,y] =  currentPic[x,y]/16

                            # Sum the values
                            sum = sum + currentPic[x,y]

                            # Store the average in the dark array
                            current[y] = sum/64
                    
                    # Reset the sum after terminating the loop     
                    sum = 0

                    # Start with movement equal to false
                    movement = False

                    # Number of changed elements from one image to another
                    movementLength = 0

                    # Start with checksum equal to zero
                    checksum = 0

                    # Size of the data to be sent over
                    size = 0

                    # Run a loop for array difference
                    for x in range (0,64):
                        if abs(current[x] - previous[x]) > CONSTANT:
                            
                            movementLength = movementLength + 1
                            movement = True

                    # Initializer
                    if movement == True:

                        print("Initializing the controller...")
                        
                        # Send the header
                        ser.write("Header: ")
                        ser.write(bytes(0XFF))
                        ser.write(bytes(0XFF))
                        ser.write(bytes(0X01))

                        # Add it to the checksum
                        checksum = 0XFF + 0XFF + 0X01
                        
                        # Compute the size of the entire message
                        size = HEADER_SIZE  + movementLength * DATA_SIZE + FOOTER_SIZE

                        
                        # Write the size of everything
                        ser.write(" Size: ")
                        ser.write(bytes(size))

                        # Add it to checksum again
                        checksum += size

                        for x in range (0, 64):
                            if abs(current[x] - previous[x]) > CONSTANT:

                                # ID
                                ser.write(" ID: ")
                                ser.write(bytes(x))

                                # low byte
                                ser.write(" lowbyte: ")
                                ser.write(bytes(current[x]))

                                # high byte
                                ser.write( "highbyte: " )
                                ser.write(bytes(0))

                                checksum += x + current[x]

                        # Module checksum and send it
                        checksum = checksum % 256

                        # Lastly send the checksum
                        ser.write( "checksum: ")
                        ser.write(bytes(checksum))
                        ser.write(" ----------------------- ")

                    if movement == False:
                        print ("still")
                    else:
                        movement = False
                        
                    if (i==loops - 1):
                        self.owner.done = True

                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)

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
    camera.color_effects = (128,128)
    camera.start_preview()
    time.sleep(2)
    output = ProcessOutput()
    camera.start_recording(output, format='mjpeg')
    while not output.done:
        camera.wait_recording(1)
    camera.stop_recording()
    ser.close
