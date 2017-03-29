import cv2
import numpy as np
import math
import sys
import time
from networktables import NetworkTable
import logging
logging.basicConfig(level=logging.DEBUG)

"""2016 FRC Vision testing on laptop with webcam"""
__author__ = "tedfoodlin"

# Capture video from camera
cap = cv2.VideoCapture(-1)

# HSV range values for different colors
LOWER_GREEN = np.array([40, 20, 20])
UPPER_GREEN = np.array([80, 220, 220])

# Set HSV range
LOWER_LIM = LOWER_GREEN
UPPER_LIM = UPPER_GREEN

# Mac webcam dimensions (approx)
MAC_FRAME_X = 1280
MAC_FRAME_Y = 720
MAC_FOV_ANGLE = 60
MAC_FOCAL_LENGTH = 15.118110236

# Dimensions in use
FRAME_X = MAC_FRAME_X
FRAME_Y = MAC_FRAME_Y
FOV_ANGLE = MAC_FOV_ANGLE
FOCAL_LENGTH = MAC_FOCAL_LENGTH
FRAME_CX = int(FRAME_X/2)
FRAME_CY = int(FRAME_Y/2)


def masking(lower, upper, frame):
    """Mask for specified color ranges."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask


def draw_static(img):
    """Draw references on frame."""
    # draw reference crosshairs
    cv2.line(img, (FRAME_CX, int(0.25*FRAME_Y)), (FRAME_CX, int(0.75*FRAME_Y)), (255, 0, 0), 3)
    cv2.line(img, (int(0.25*FRAME_X), FRAME_CY), (int(0.75*FRAME_X), FRAME_CY), (255, 0, 0), 3)


def polygon(c):
    """Remove concavities from a contour and turn it into a polygon."""
    hull = cv2.convexHull(c)
    epsilon = 0.02 * cv2.arcLength(hull, True)
    goal = cv2.approxPolyDP(hull, epsilon, True)
    return goal


def calc_center(M):
    """Detect the center given the moment of a contour."""
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx, cy


def calc_horiz_angle(error):
    """Calculates the horizontal angle from pixel error"""
    return math.atan(error / FOCAL_LENGTH)


def is_aligned(error):
    """Check if shooter is aligned and ready to shoot."""
    if 1 > error > -1:
        return True
    else:
        return False


def report_command(error):
    """Testing - show robot commands in terminal."""
    if 1 > error > -1:
        print("X Aligned")
    else:
        if error > 10:
            print("Turn Right")
        elif error < -10:
            print("Turn Left")


def report_y(cy):
    """Report state of y to terminal."""
    # Maybe useful but not necessary if you have a nice set shooter angle
    if FRAME_CY + 10 > cy > FRAME_CY - 10:
        print("Y Aligned")
    else:
        if cy > FRAME_CY + 10:
            print("Aim Lower")
        elif cy < FRAME_CY - 10:
            print("Aim Higher")


def main():
    # network table setup
    NetworkTable.setIPAddress("127.0.0.1")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    SmartDashboard = NetworkTable.getTable("NerdyVision")

    # adjust camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_X)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_Y)
    # cap.set(cv2.CAP_PROP_FPS,30)
    cap.set(cv2.CAP_PROP_EXPOSURE, -8.0)

    # set up FPS list and iterator
    times = [0] * 25
    time_idx = 0
    time_start = time.time()
    camfps = 0

    while 687:
        ret, frame = cap.read()

        # compute FPS information
        time_end = time.time()
        times[time_idx] = time_end - time_start
        time_idx += 1
        if time_idx >= len(times):
            camfps = 1 / (sum(times) / len(times))
            time_idx = 0
        if time_idx > 0 and time_idx % 5 == 0:
            camfps = 1 / (sum(times) / len(times))
        time_start = time_end
        print("FPS: " + str(camfps))
        print("Time: " + str(time.time()))

        # init values (for x)
        angle_to_turn = 0
        aligned = False

        # gaussian blur to remove noise
        blur = cv2.GaussianBlur(frame, (11, 11), 0)

        # remove everything but specified color
        res, mask = masking(LOWER_LIM, UPPER_LIM, blur)

        # draw references
        draw_static(res)

        # find contour of goal
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour (closest goal) in the mask
            c = max(cnts, key=cv2.contourArea)

            # make sure the largest contour is significant
            area = cv2.contourArea(c)
            if area > 1500:
                # make suggested contour into a polygon
                goal = polygon(c)

                # make sure goal contour has 4 sides
                if len(goal) == 4:
                    # draw the contour
                    cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)

                    # calculate centroid
                    M = cv2.moments(goal)
                    if M['m00'] > 0:
                        cx, cy = calc_center(M)
                        center = (cx, cy)

                        # draw centroid
                        cv2.circle(res, center, 5, (255, 0, 0), -1)

                        # calculate error in degrees
                        error = cx - FRAME_CX
                        angle_to_turn = calc_horiz_angle(error)
                        print("Angle to turn: " + str(angle_to_turn))

                        # check if shooter is aligned
                        aligned = is_aligned(angle_to_turn)
                        print("Aligned: " + str(aligned))

                        report_command(error)

        # results
        cv2.imshow("NerdyVision", res)
        try:
            # send to network tables
            SmartDashboard.putNumber('ANGLE_TO_TURN', angle_to_turn)
            SmartDashboard.putBoolean('IS_ALIGNED', aligned)
        except:
            print("DATA NOT SENDING...")
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
