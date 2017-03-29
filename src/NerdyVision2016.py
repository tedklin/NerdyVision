import logging
import os

import cv2
from imutils.video import WebcamVideoStream
from networktables import NetworkTable

import NerdyConstants
import NerdyFunctions

logging.basicConfig(level=logging.DEBUG)

"""2016 FRC Vision Processing on Raspberry Pi with Microsoft Lifecam"""
__author__ = "tedfoodlin"

if not os.path.isdir("/tmp/stream"):
   os.makedirs("/tmp/stream")

cap = WebcamVideoStream(src=-1).start()


def main():

    NetworkTable.setIPAddress("roboRIO-687-FRC.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    SmartDashboard = NetworkTable.getTable("NerdyVision")

    while 687:

        angle_to_turn = 0
        aligned = False

        frame = cap.read()

        NerdyFunctions.draw_static(res)
        cv2.imwrite("/tmp/stream/img.jpg", res)

        blur = cv2.GaussianBlur(frame, (11, 11), 0)
        # kernel = np.ones((5, 5), np.uint8)
        # erosion = cv2.erode(frame, kernel, iterations=1)
        # dilation = cv2.dilate(erosion, kernel, iterations=1)
        res, mask = NerdyFunctions.mask(NerdyConstants.LOWER_GREEN, NerdyConstants.UPPER_GREEN, blur)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > NerdyConstants.MIN_GOAL_AREA:
                goal = NerdyFunctions.polygon(c, 0.025)

                if len(goal) == 4:
                    cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)
                    M = cv2.moments(goal)

                    if M['m00'] > 0:
                        cx, cy = NerdyFunctions.calc_center(M)
                        center = (cx, cy)
                        cv2.circle(res, center, 5, (255, 0, 0), -1)

                        error = cx - NerdyConstants.FRAME_CX
                        angle_to_turn = NerdyFunctions.calc_horiz_angle(error)
                        print("ANGLE_TO_TURN" + str(angle_to_turn))
                        aligned = NerdyFunctions.is_aligned(angle_to_turn)
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
