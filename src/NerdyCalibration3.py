import cv2
import numpy as np
import math
import os
import NerdyConstants
import NerdyFunctions

"""FRC Vision Target Countour Filtering Calibration (Trackbars)"""
__author__ = "tedlin"

# Capture video from camera
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, NerdyConstants.FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, NerdyConstants.FRAME_Y)

cv2.namedWindow('result')


def placeholder(x):
    pass


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

    max_area = NerdyConstants.FRAME_X * NerdyConstants.FRAME_Y
    min_area = 0
    target_area = 0
    cv2.createTrackbar('minimum target area', 'result', min_area, max_area, placeholder)
    cv2.createTrackbar('maximum target area', 'result', min_area, max_area, placeholder)

    while 687:
        ret, frame = cap.read()

        blur = cv2.GaussianBlur(frame, (11, 11), 0)
        #kernel = np.ones((5, 5), np.uint8)
        #erosion = cv2.erode(frame, kernel, iterations=1)
        #dilation = cv2.dilate(erosion, kernel, iterations=1)
        res, mask = NerdyFunctions.mask(NerdyConstants.LOWER_GREEN, NerdyConstants.UPPER_GREEN, blur)

        _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        min_area = cv2.getTrackbarPos('minimum target area', 'result')
        max_area = cv2.getTrackbarPos('maximum target area', 'result')

        if min_area < max_area:
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > min_area and area < max_area:
                    goal = NerdyFunctions.polygon(c, 0)
                    if len(goal) == 4:
                        cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)
                        M = cv2.moments(goal)
                        target_area = area

        print("min target area: " + str(min_area))
        print("max target area: " + str(max_area))
        print("target area: " + str(target_area))

        cv2.imshow("NerdyVision", res)

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
