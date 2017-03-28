import cv2
import numpy as np
from networktables import NetworkTable
import os
import logging
logging.basicConfig(level=logging.DEBUG)

"""2017 FRC Gear Vision Targeting on Raspberry Pi with Microsoft Lifecam"""
__author__ = "tedfoodlin"

if not os.path.isdir("/tmp/stream"):
   os.makedirs("/tmp/stream")

cap = cv2.VideoCapture(-1)

FRAME_X = 640
FRAME_Y = 480
FRAME_CX = int(FRAME_X/2)
FRAME_CY = int(FRAME_Y/2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_Y)
cap.set(cv2.CAP_PROP_EXPOSURE, -8.0)

FOV_ANGLE = 59.02039664
DEGREES_PER_PIXEL = FOV_ANGLE / FRAME_X

MIN_AREA = 15000
MAX_AREA = 50000

NetworkTable.setIPAddress("roboRIO-687-FRC.local")
NetworkTable.setClientMode()
NetworkTable.initialize()
SmartDashboard = NetworkTable.getTable("NerdyVision")

while 687:
    ret, frame = cap.read()

    cv2.line(frame, (FRAME_CX, int(0.25*FRAME_Y)),
             (FRAME_CX, int(0.75*FRAME_Y)), (0, 0, 255), 3)
    cv2.line(frame, (int(0.25*FRAME_X), FRAME_CY),
             (int(0.75*FRAME_X), FRAME_CY), (0, 0, 255), 3)
    cv2.imwrite("/tmp/stream/img.jpg", frame)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([40, 50, 50]),
                       np.array([60, 250, 300]))
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)
    center = None

    if len(cnts) > 1:
        centers_x = [0]
        centers_y = [0]

        for i in range(len(cnts)):
            c = cnts[i]
            area = cv2.contourArea(c)
            if MIN_AREA < area < MAX_AREA:
                hull = cv2.convexHull(c)
                epsilon = 0.02 * cv2.arcLength(hull, True)
                goal = cv2.approxPolyDP(hull, epsilon, True)

                cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)

                M = cv2.moments(goal)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    center = (cx, cy)

                    cv2.circle(res, center, 5, (255, 0, 0), -1)

                    centers_x.append(cx)
                    centers_y.append(cy)

        if len(centers_x) == 3 and len(centers_y) == 3:
            target_x = (centers_x[1] + centers_x[2])/2
            target_y = (centers_y[1] + centers_y[2])/2
            target = (target_x, target_y)
            cv2.circle(res, target, 5, (0, 255, 0), -1)
            print(target_x)
            print(target_y)

            error = target_x - FRAME_CX
            angle_to_turn = error * DEGREES_PER_PIXEL
            aligned = 1 > angle_to_turn > -1

    cv2.imshow("NerdyVision", res)
    try:
        SmartDashboard.putNumber('ANGLE_TO_TURN', angle_to_turn)
        SmartDashboard.putBoolean('IS_ALIGNED', aligned)
        print("DATA SENDING")
    except:
        print("DATA NOT SENDING...")

    cv2.imshow("NerdyVision", res)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
