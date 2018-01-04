import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib import animation

import NerdyConstants

"""FRC Vision Target HSV Color Calibration (Trackbars)"""
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

    lower_h, lower_s, lower_v = 16, 67, 118
    upper_h, upper_s, upper_v = 33, 148, 254

    cv2.createTrackbar('lower h', 'result', 0, 179, placeholder)
    cv2.createTrackbar('lower s', 'result', 0, 255, placeholder)
    cv2.createTrackbar('lower v', 'result', 0, 255, placeholder)
    cv2.createTrackbar('upper h', 'result', 0, 179, placeholder)
    cv2.createTrackbar('upper s', 'result', 0, 255, placeholder)
    cv2.createTrackbar('upper v', 'result', 0, 255, placeholder)

    while 687:
        ret, frame = cap.read()

        fig = plt.figure(1)
        fig2 = plt.figure(2)

        # cv2.imshow('Raw', frame)
        plot = fig.add_subplot(1, 1, 1)
        plot.imshow(frame, animated=True)
        animation.FuncAnimation(fig, frame, interval=0)
        fig.show()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_h = cv2.getTrackbarPos('lower h', 'result')
        lower_s = cv2.getTrackbarPos('lower s', 'result')
        lower_v = cv2.getTrackbarPos('lower v', 'result')
        upper_h = cv2.getTrackbarPos('upper h', 'result')
        upper_s = cv2.getTrackbarPos('upper s', 'result')
        upper_v = cv2.getTrackbarPos('upper v', 'result')

        if (lower_h < upper_h) & (lower_s < upper_s) & (lower_v < upper_v):
            lower_green = np.array([lower_h, lower_s, lower_v])
            upper_green = np.array([upper_h, upper_s, upper_v])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            # cv2.imshow('Filtered', result)
            filtered = fig2.add_subplot(1, 1, 1)
            filtered.imshow(result, animated=True)
            animation.FuncAnimation(fig2, result, interval=0)
            fig2.show()
        else:
            print("ERROR: make sure lower limit is lower than upper limit")

        print("lower HSV limit: " + str(lower_h) + ", " + str(lower_s) + ", " + str(lower_v))
        print("upper HSV limit: " + str(upper_h) + ", " + str(upper_s) + ", " + str(upper_v))

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
