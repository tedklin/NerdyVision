import cv2
import os

"""Raspberry Pi / Microsoft Lifecam MJPG Streamer adaptation with crosshairs"""

if not os.path.isdir("/tmp/stream"):
   os.makedirs("/tmp/stream")

cap = cv2.VideoCapture(0)

FRAME_X = 320
FRAME_Y = 240
FRAME_CX = int(FRAME_X/2)
FRAME_CY = int(FRAME_Y/2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_Y)

while 687:
    ret, frame = cap.read()

    cv2.line(frame, (FRAME_CX, int(0.25*FRAME_Y)), (FRAME_CX, int(0.75*FRAME_Y)), (0, 255, 0), 3)
    cv2.line(frame, (int(0.25*FRAME_X), FRAME_CY), (int(0.75*FRAME_X), FRAME_CY), (0, 255, 0), 3)
    cv2.imwrite("/tmp/stream/img.jpg", frame)

    cv2.waitKey(1)
