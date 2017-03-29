import cv2
import numpy as np
import math
import sys
import time
from networktables import NetworkTable
import logging
logging.basicConfig(level=logging.DEBUG)

"""2017 FRC Vision testing on laptop with Microsoft Lifecam"""
__author__ = "tedfoodlin"

# Capture video from camera (0 for laptop webcam, 1 for USB camera)
cap = cv2.VideoCapture(-1)

# for sample image testing (images from 1 to 32)
sample_image = 32

# HSV range values for testing sample images
SAMPLE_LOWER = np.array([80, 70, 80])
SAMPLE_UPPER = np.array([100, 300, 300])

# HSV range values for green highlighter
LOWER_GREEN = np.array([40, 50, 50])
UPPER_GREEN = np.array([60, 250, 300])

# Set modes (if you don't want user input)
SHOOTING = True
GEARS = False

# Set HSV range
LOWER_LIM = LOWER_GREEN
UPPER_LIM = UPPER_GREEN

# Dimensions in use (Microsoft Lifecam HD-3000)
FRAME_X = 640
FRAME_Y = 480
FOV_ANGLE = 59.02039664
DEGREES_PER_PIXEL = FOV_ANGLE / FRAME_X
FRAME_CX = int(FRAME_X/2)
FRAME_CY = int(FRAME_Y/2)

# Gear dimensions
MIN_AREA = 15000
MAX_AREA = 50000


def check_modes():
    """Check which modes are on based on user input."""
    shooting = False
    gears = False
    if raw_input("Shooting mode on? (y/n)") == "y":
        shooting = True
    if raw_input("Gears mode on? (y/n)") == "y":
        gears = True
    return shooting, gears


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


def polygon(c, epsil):
    """Remove concavities from a contour and turn it into a polygon."""
    hull = cv2.convexHull(c)
    epsilon = epsil * cv2.arcLength(hull, True)
    goal = cv2.approxPolyDP(hull, epsilon, True)
    return goal


def calc_center(M):
    """Detect the center given the moment of a contour."""
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx, cy


def calc_horiz_angle(error):
    """Calculates the horizontal angle from pixel error"""
    return error * DEGREES_PER_PIXEL


def is_aligned(angle_to_turn):
    """Check if shooter is aligned and ready to shoot."""
    if 1 > angle_to_turn > -1:
        return True
    else:
        return False


def avg(x1, x2):
    """"Take average of 2 numbers"""
    sum = x1 + x2
    return sum / 2


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
    # set modes (default without user input)
    shooting = SHOOTING
    gears = GEARS

    # turn on modes specified by user
    # uncomment next line if this feature is desired
    # shooting, gears = check_modes()

    # network table setup
    NetworkTable.setIPAddress("roboRIO-687-FRC.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    SmartDashboard = NetworkTable.getTable("NerdyVision")

    # adjust camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_X)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_Y)
    # cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_EXPOSURE, -8.0)

    # set up FPS list and iterator
    times = [0] * 25
    time_idx = 0
    time_start = time.time()
    camfps = 0

    while 687:
        ret, frame = cap.read()

        # the next 2 lines are for sample image testing for shooting
        #frame = cv2.imread("sample_images/LED_Boiler/" + str(sample_image) + ".jpg")
        #print(sample_image)

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
        _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        center = None

        if shooting:
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour (closest goal) in the mask
                c = max(cnts, key=cv2.contourArea)

                # make sure the largest contour is significant
                area = cv2.contourArea(c)

                if area > 300:
                    goal = polygon(c, 0)

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
                        print("ANGLE TO TURN " + str(angle_to_turn))

                        # check if shooter is aligned
                        aligned = is_aligned(angle_to_turn)
                        print("ALIGNED " + str(aligned))

                        report_command(error)
                        report_y(cy)

        elif gears:
            # only proceed if at least two contours (two blocks around peg) was found
            if len(cnts) > 1:
                centers_x = [0]
                centers_y = [0]

                # find the two blocks in the mask based on areas
                for i in range(len(cnts)):
                    c = cnts[i]
                    area = cv2.contourArea(c)
                    if MIN_AREA < area < MAX_AREA:
                        goal = polygon(c, 0.02)

                        # draw the contour
                        cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)

                        M = cv2.moments(goal)
                        if M['m00'] > 0:
                            cx, cy = calc_center(M)
                            center = (cx, cy)

                            # draw centroid
                            cv2.circle(res, center, 5, (255, 0, 0), -1)

                            centers_x.append(cx)
                            centers_y.append(cy)

                # calculate center of two contours (blocks next to peg)
                if len(centers_x) == 3 and len(centers_y) == 3:
                    target_x = avg(centers_x[1], centers_x[2])
                    target_y = avg(centers_y[1], centers_y[2])
                    target = (target_x, target_y)
                    cv2.circle(res, target, 5, (0, 255, 0), -1)
                    print(target_x)
                    print(target_y)

                    # calculate angle to turn
                    error = target_x - FRAME_CX
                    angle_to_turn = calc_horiz_angle(error)
                    print("ANGLE TO TURN " + str(angle_to_turn))

                    # check if gear mechanism is aligned
                    aligned = is_aligned(angle_to_turn)
                    print("ALIGNED " + str(aligned))

                    report_command(error)

        # results
        cv2.imshow("NerdyVision", res)

        try:
            # send to network tables
            SmartDashboard.putNumber('ANGLE_TO_TURN', angle_to_turn)
            SmartDashboard.putBoolean('IS_ALIGNED', aligned)
            print("DATA SENDING")
        except:
            print("DATA NOT SENDING...")
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
