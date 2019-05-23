import logging
import os
import time
import cv2
import numpy as np
import math
# from networktables import NetworkTable
import NerdyConstants
import NerdyFunctions
# logging.basicConfig(level=logging.DEBUG)

"""2016 FRC High Goal Image Processing on Raspberry Pi with Microsoft Lifecam"""
__author__ = "tedlin"

cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, NerdyConstants.FRAME_X)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, NerdyConstants.FRAME_Y)


# checks if the contour is tilted to the right
def get_orientation(contour_corners): 
    # contour_ab = contour_corners[0][1] - contour_corners[1][1]
    # contour_ad = contour_corners[0][1] - contour_corners[3][1]
    
    # # 1 is oriented left, 2 is right, 3 is vertical
    # if(contour_ab < contour_ad):
    #     return 1
    # elif(contour_ab > contour_ad):
    #     return 2
    # elif(contour_corners[2][0] == contour_corners[1][0] or contour_corners[2][1] == contour_corners[2][1]):
    #     return 3

    dy_1 = contour_corners[0][1] - contour_corners[1][1]
    dx_1 = contour_corners[0][0] - contour_corners[1][0]
    hypo_1 = math.sqrt(math.pow(dy_1, 2) + math.pow(dx_1, 2))

    dy_2 = contour_corners[1][1] - contour_corners[2][1]
    dx_2 = contour_corners[1][0] - contour_corners[2][0]
    hypo_2 = math.sqrt(math.pow(dy_2, 2) + math.pow(dx_2, 2))

    if hypo_2 > hypo_1:
        return 1
    elif hypo_1 > hypo_2:
        return 2
    else:
        return 3


def main():

    # brightness adjusted, used to be 30, now is 70
    # os.system("v4l2-ctl -d /dev/video0 "
    #           "-c brightness=70 "
    #           "-c contrast=10 "
    #           "-c saturation=100 "
    #           "-c white_balance_temperature_auto=0 "
    #           "-c power_line_frequency=2 "
    #           "-c white_balance_temperature=4500 "
    #           "-c sharpness=25 "
    #           "-c backlight_compensation=0 "
    #           "-c exposure_auto=1 "
    #           "-c exposure_absolute=5 "
    #           "-c pan_absolute=0 "
    #           "-c tilt_absolute=0 "
    #           "-c zoom_absolute=0")

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
        res, mask = NerdyFunctions.mask(NerdyConstants.LOWER_LIMIT, NerdyConstants.UPPER_LIMIT, dilation)

        _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > NerdyConstants.MIN_GOAL_AREA:
                goal = NerdyFunctions.polygon(c, 0.025)

                if len(goal) == 4:
                    M = cv2.moments(goal)

                    if M['m00'] > 0:
                        cv_cx, cv_cy = NerdyFunctions.calc_center(M)
                        center = (cv_cx, cv_cy)
                        # DISPLAY BLOCK REMEMBER TO COMMENT OUT
                        cv2.circle(res, center, 5, (255, 0, 0), -1) # remember to comment out
                        
                        real_cx = cv_cx - (NerdyConstants.FRAME_CX)
                        real_cy = (NerdyConstants.FRAME_CY) - cv_cy
                        center = (real_cx, real_cy)
                        print(center)

                        rect = cv2.minAreaRect(goal)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)

                        orientation = get_orientation(box)
                        print(orientation)
                        if (orientation == 1):
                            dy = abs(box[2][1] - box[1][1])
                            dx = box[2][0] - box[1][0]
                            angle = 90 - math.degrees(math.atan2(dy, dx))
                            slope = dy / dx * 1.0
                            y_int = slope * -real_cx + real_cy
                        elif (orientation == 2):
                            dy = abs(box[1][1] - box[0][1])
                            dx = box[1][0] - box[0][0]
                            angle = 90 - math.degrees(math.atan2(dy, dx))
                            slope = dy / dx * 1.0
                            y_int = slope * -real_cx + real_cy
                        else:
                            dy = 0
                            dx = 1
                            angle = 0
                            slope = 0
                            y_int = 99999
                        if math.isinf(y_int):
                            y_int = 99999
                        else:
                            y_int = int(y_int)

                        print(dy)
                        print(dx)
                        print(angle)
                        print(slope)
                        print(y_int)
                        print()

                        cv_y_int = (NerdyConstants.FRAME_CX, (NerdyConstants.FRAME_CY - y_int))
                        cv2.circle(res, cv_y_int, 5, (0, 255, 0), -1) # remember to comment out

                        # DISPLAY BLOCK REMEMBER TO COMMENT OUT
                        # cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)
                        cv2.drawContours(res,[box],0,(255,0,0),2)


        # DISPLAY BLOCK REMEMBER TO COMMENT OUT
        NerdyFunctions.draw_static(res) # this just draws crosshairs
        cv2.imshow("NerdyVision", res)

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
