import logging
import os
import time
import cv2
import numpy as np
from CameraStream import CameraStream
from networktables import NetworkTable
import NerdyConstants
import NerdyFunctions
logging.basicConfig(level=logging.DEBUG)

"""2016 FRC Vision Processing on Raspberry Pi with Microsoft Lifecam"""
__author__ = "tedfoodlin"

if not os.path.isdir("/tmp/stream"):
   os.makedirs("/tmp/stream")

#cap = CameraStream(src=-1).start()
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, NerdyConstants.FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, NerdyConstants.FRAME_Y)


def main():

    # brightness adjusted, used to be 30, now is 70
    os.system("v4l2-ctl -d /dev/video0 "
              "-c brightness=70 "
              "-c contrast=10 "
              "-c saturation=100 "
              "-c white_balance_temperature_auto=0 "
              "-c power_line_frequency=2 "
              "-c white_balance_temperature=4500 "
              "-c sharpness=25 "
              "-c backlight_compensation=0 "
              "-c exposure_auto=1 "
              "-c exposure_absolute=5 "
              "-c pan_absolute=0 "
              "-c tilt_absolute=0 "
              "-c zoom_absolute=0")

    NetworkTable.setIPAddress("roboRIO-687-FRC.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    table = NetworkTable.getTable("NerdyVision")
    print("NetworkTables initialized")

    angle_to_turn = 0

    while 687:

        aligned = False
        previous_angle_to_turn = angle_to_turn

        ret, frame = cap.read()
        capture_time = time.time()

        # blur = cv2.GaussianBlur(frame, (11, 11), 0)
        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(frame, kernel, iterations=1)
        dilation = cv2.dilate(erosion, kernel, iterations=1)
        res, mask = NerdyFunctions.mask(NerdyConstants.LOWER_GREEN, NerdyConstants.UPPER_GREEN, dilation)

        _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

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
                        print("ANGLE_TO_TURN: " + str(angle_to_turn))
                        aligned = NerdyFunctions.is_aligned(angle_to_turn)
                        print("IS_ALIGNED: " + str(aligned))

                        processed_time = time.time()
                        delta_time = processed_time - capture_time
                        print("PROCESSED_TIME: " + str(delta_time))

        # Has to be commented out because ssh doesn't allow opencv windows open
        # NerdyFunctions.draw_static(res)
        # cv2.imshow("NerdyVision", res)
        try:
            table.putBoolean('IS_ALIGNED', aligned)
            if previous_angle_to_turn != angle_to_turn:
                table.putNumber('ANGLE_TO_TURN', angle_to_turn)
                table.putNumber('PROCESSED_TIME', delta_time)
            else :
                table.putNumber('ANGLE_TO_TURN', 0)
                table.putNumber('PROCESSED_TIME', 0)
            table.putBoolean('VISION_ON', True)
        except:
            print("DATA NOT SENDING...")
            table.putBoolean('IS_ALINGED', False)
            table.putNumber('ANGLE_TO_TURN', 0)
            table.putNumber('PROCESSED_TIME', 0)
            table.putBoolean('VISION_ON', False)

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
