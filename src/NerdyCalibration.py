import cv2
import numpy as np
import math
import os
import NerdyConstants

"""FRC Vision Target Calibration (Box)"""
__author__ = "tedfoodlin"

# Capture video from camera (0 for laptop webcam, 1 for USB camera)
cap = cv2.VideoCapture(-1)

# Calibration box dimensions
CAL_AREA = 1600
CAL_SIZE = int(math.sqrt(CAL_AREA))
CAL_UP = NerdyConstants.FRAME_CY + (CAL_SIZE / 2)
CAL_LO = NerdyConstants.FRAME_CY - (CAL_SIZE / 2)
CAL_R = NerdyConstants.FRAME_CX - (CAL_SIZE / 2)
CAL_L = NerdyConstants.FRAME_CX + (CAL_SIZE / 2)
CAL_UL = (CAL_L, CAL_UP)
CAL_LR = (CAL_R, CAL_LO)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, NerdyConstants.FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, NerdyConstants.FRAME_Y)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
cap.set(cv2.CAP_PROP_CONTRAST, 1)
cap.set(cv2.CAP_PROP_SATURATION, 1)

os.system("v4l2-ctl -d /dev/video-1 -c exposure_auto=1, "
          "exposure_absolute=5, "
          "white_balance_temperature_auto=0, "
          "white_balance_temperature=8000")


def main():
    while 687:
        ret, frame = cap.read()

        cv2.rectangle(frame, CAL_UL, CAL_LR, (0, 255, 0), thickness=1)
        roi = frame[CAL_LO:CAL_UP, CAL_R:CAL_L]
        average_color_per_row = np.average(roi, axis=0)
        average_color = np.average(average_color_per_row, axis=0)
        average_color = np.uint8([[average_color]])
        hsv = cv2.cvtColor(average_color, cv2.COLOR_BGR2HSV)

        print(np.array_str(hsv))
        cv2.imshow("NerdyCalibration", frame)
        cv2.imwrite("/tmp/stream/img.jpg", frame)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
