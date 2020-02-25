# MotionDetectorAnimatlab
Simple motion detection algorithm that transmits data to Animatlab. 

Because OpenCV is a pain in the a** to get to work with Python 3 on Jessie, 
the blob detector needs to be run seperately with Python2 to get the correct center of the camera before
running the movement detector with Python3.

Steps to get this to send the correct data over serial:

1. Check that the test sample covers the entire screen with camera_setup.py 
2. Run the blob_detector.py in Python 2 and copy the printed coordinates.
3. Run python3 movement_detector.py with the correct coordinates.
4. The code should run and print the speed of data being sent over without an issue. 