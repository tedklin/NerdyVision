# import the necessary packages
from __future__ import print_function
from imutils.video import WebcamVideoStream
import cv2

"""Nerdy Optimized Streaming Test"""

vs = WebcamVideoStream(src=-1).start()
print("initialized")
frameNumber = 0

while 687:
    frame = vs.read()

    cv2.imshow("Frame", frame)
    print(frameNumber)
    frameNumber += 1
