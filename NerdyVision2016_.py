import cv2
import numpy as np
import math
from networktables import NetworkTable
import os
import logging
logging.basicConfig(level=logging.DEBUG)

"""2016 FRC Vision Processing on Raspberry Pi with Microsoft Lifecam"""
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

NetworkTable.setIPAddress("roboRIO-687-FRC.local")
NetworkTable.setClientMode()
NetworkTable.initialize()
SmartDashboard = NetworkTable.getTable("NerdyVision")

LOWER_LIM = np.array([40, 20, 20])
UPPER_LIM = np.array([80, 220, 220])

MIN_AREA = 1500


def masking(lower, upper, frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask


def draw_static(img):
    cv2.line(img, (FRAME_CX, int(0.25*FRAME_Y)), (FRAME_CX, int(0.75*FRAME_Y)), (0, 255, 0), 3)
    cv2.line(img, (int(0.25*FRAME_X), FRAME_CY), (int(0.75*FRAME_X), FRAME_CY), (0, 255, 0), 3)


def polygon(c):
    hull = cv2.convexHull(c)
    epsilon = 0.025 * cv2.arcLength(hull, True)
    goal = cv2.approxPolyDP(hull, epsilon, True)
    return goal


def calc_center(M):
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx, cy


def calc_horiz_angle(error):
    #return math.atan(error / FOCAL_LENGTH)
    return error * DEGREES_PER_PIXEL


def is_aligned(error):
    if 1 > error > -1:
        return True
    else:
        return False


def report_command(error):
    if 1 > error > -1:
        print("X Aligned")
    else:
        if error > 10:
            print("Turn Right")
        elif error < -10:
            print("Turn Left")


def report_y(cy):
    if FRAME_CY + 10 > cy > FRAME_CY - 10:
        print("Y Aligned")
    else:
        if cy > FRAME_CY + 10:
            print("Aim Lower")
        elif cy < FRAME_CY - 10:
            print("Aim Higher")


def main():
    while 687:

        angle_to_turn = 0
        aligned = False

        ret, frame = cap.read()
        # blur = cv2.GaussianBlur(frame, (11, 11), 0)
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(frame, kernel, iterations=1)
        dilation = cv2.dilate(erosion, kernel, iterations=1)
        res, mask = masking(LOWER_LIM, UPPER_LIM, dilation)

        draw_static(res)
        cv2.imwrite("/tmp/stream/img.jpg", res)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                goal = polygon(c)

                if len(goal) == 4:
                    cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)
                    M = cv2.moments(goal)

                    if M['m00'] > 0:
                        cx, cy = calc_center(M)
                        center = (cx, cy)
                        cv2.circle(res, center, 5, (255, 0, 0), -1)

                        error = cx - FRAME_CX
                        angle_to_turn = calc_horiz_angle(error)
                        print("ANGLE_TO_TURN" + str(angle_to_turn))
                        aligned = is_aligned(angle_to_turn)
                        print("IS_ALIGNED: " + str(aligned))

        cv2.imshow("NerdyVision", res)
        try:
            SmartDashboard.putNumber('ANGLE_TO_TURN', angle_to_turn)
            SmartDashboard.putBoolean('IS_ALIGNED', aligned)
        except:
            print("DATA NOT SENDING...")

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
