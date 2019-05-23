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

width_inches = 4
fc_to_real_center_inches = 8

def pixels_to_inches(pixels, width_pixels):
    if width_pixels == 0:
        return 0
    else:
        return pixels * width_inches / width_pixels


def inches_to_pixels(inches, width_pixels):
    if width_pixels == 0:
        return 0
    else: 
        return inches * width_pixels / width_inches


def main():

    while 687:

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
            goal = NerdyFunctions.polygon(c, 0.025)
            M = cv2.moments(goal)

            if area > NerdyConstants.MIN_GOAL_AREA and len(goal) == 4 and M['m00'] > 0:
                cv_cx, cv_cy = NerdyFunctions.calc_center(M)
                center = (cv_cx, cv_cy)

                # DISPLAY BLOCK REMEMBER TO COMMENT OUT
                cv2.circle(res, center, 5, (255, 0, 0), -1) # remember to comment out
                
                real_cx = cv_cx - (NerdyConstants.FRAME_CX)
                real_cy = (NerdyConstants.FRAME_CY) - cv_cy
                center = (real_cx, real_cy)
                # print(center)

                rect = cv2.minAreaRect(goal)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # orientation = get_orientation(box)
                # print(orientation)
                dy_1 = box[0][1] - box[1][1]
                dx_1 = box[0][0] - box[1][0]
                hypo_1 = math.sqrt(math.pow(dy_1, 2) + math.pow(dx_1, 2))

                dy_2 = box[1][1] - box[2][1]
                dx_2 = box[1][0] - box[2][0]
                hypo_2 = math.sqrt(math.pow(dy_2, 2) + math.pow(dx_2, 2))

                if (hypo_2 > hypo_1):
                    dy = abs(box[2][1] - box[1][1])
                    dx = box[2][0] - box[1][0]
                    angle = 90 - math.degrees(math.atan2(dy, dx))
                    slope = dy / dx * 1.0
                    y_int = slope * -real_cx + real_cy
                    width_pixels = hypo_1
                elif (hypo_1 > hypo_2):
                    dy = abs(box[1][1] - box[0][1])
                    dx = box[1][0] - box[0][0]
                    angle = 90 - math.degrees(math.atan2(dy, dx))
                    slope = dy / dx * 1.0
                    y_int = slope * -real_cx + real_cy
                    width_pixels = hypo_2
                else:
                    dy = 0
                    dx = 1
                    angle = 0
                    slope = 0
                    y_int = 99999
                    width_pixels = 0

                if math.isinf(y_int):
                    y_int = 99999
                else:
                    y_int = int(y_int)

                # print(dy)
                # print(dx)
                # print(angle)
                # print(slope)
                # print(y_int)
                print()

                cv_y_int = (NerdyConstants.FRAME_CX, (NerdyConstants.FRAME_CY - y_int))
                cv2.circle(res, cv_y_int, 5, (0, 255, 0), -1) # remember to comment out

                # DISPLAY BLOCK REMEMBER TO COMMENT OUT
                # cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)
                cv2.drawContours(res,[box],0,(255,0,0),2)

                # we need to send real_cx, real_cy, angle, y_int
                real_cx_inches = pixels_to_inches(real_cx, width_pixels)
                real_cy_inches = pixels_to_inches(real_cx, width_pixels)
                y_int_inches = pixels_to_inches(y_int, width_pixels)

                print(real_cx_inches)
                print(real_cy_inches)
                print(y_int_inches)

                offset_inches = inches_to_pixels(fc_to_real_center_inches, width_pixels)
                real_cy_inches = real_cy_inches + offset_inches
                y_int_inches = y_int_inches + offset_inches

                # stuff to send
                # print(offset_inches)
                # print(real_cx_inches)
                # print(real_cy_inches)
                # print(y_int_inches)
                # print(angle)


        # DISPLAY BLOCK REMEMBER TO COMMENT OUT
        NerdyFunctions.draw_static(res) # this just draws crosshairs
        cv2.imshow("NerdyVision", res)

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
