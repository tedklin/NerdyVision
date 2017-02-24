import cv2

"""Raspberry Pi / Microsoft Lifecam MJPG Streamer adaptation (with crosshairs)"""

cap = cv2.VideoCapture(0)

FRAME_X = 640
FRAME_Y = 480

FRAME_CX = FRAME_X/2
FRAME_CY = FRAME_Y/2

# adjust camera settings
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_Y)

while 687:
    ret, frame = cap.read()

    cv2.line(frame, (FRAME_CX, int(0.25*FRAME_Y)), (FRAME_CX, int(0.75*FRAME_Y)), (0, 255, 0), 3)
    cv2.line(frame, (int(0.25*FRAME_X), FRAME_CY), (int(0.75*FRAME_X), FRAME_CY), (0, 255, 0), 3)
    cv2.imwrite("/tmp/stream/img.jpg", frame)
