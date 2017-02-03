import cv2
import numpy as np
import urllib
import math
from networktables import NetworkTable
import logging
logging.basicConfig(level=logging.DEBUG)

"""2016 FRC Vision Processing on Driver Station with Axis"""
__author__ = "tedfoodlin"

# HSV range values for green
LOWER_LIM = np.array([40, 20, 20])
UPPER_LIM = np.array([80, 220, 220])

# Axis M1013 dimensions
FRAME_X = 640
FRAME_Y = 480
FOV_ANGLE = 67
FOCAL_LENGTH = 10.582677165    # 2.80 mm in pixels
FRAME_CX = FRAME_X / 2
FRAME_CY = FRAME_Y / 2


def masking(lower, upper, frame):
    """Mask for specified color ranges."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask


def draw_static(img):
    """Draw references on frame."""
    # draw center of frame
    cv2.circle(img, (FRAME_CX, FRAME_CY), 5,
               (0, 0, 255), -1)
    # draw reference line for x position
    cv2.line(img, (FRAME_CX, 0), (FRAME_CX, FRAME_Y),
             (0, 0, 255), 2)


def polygon(c):
    """Remove concavities from a contour and turn it into a polygon."""
    hull = cv2.convexHull(c)
    epsilon = 0.025 * cv2.arcLength(hull, True)
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
    NetworkTable.setIPAddress("roboRIO-687-FRC.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    table = NetworkTable.getTable("NerdyVision")

    # stream from axis camera setup
    stream = urllib.urlopen('http://10.06.87.11/mjpg/video.mjpg')
    bytes = ''

    while 687:

        # get frame from stream
        bytes += stream.read(16384)
        b = bytes.rfind('\xff\xd9')
        a = bytes.rfind('\xff\xd8', 0, b-1)

        if a != -1 and b != -1:
            jpg = bytes[a:b+2]
            bytes = bytes[b+2:]
            frame = cv2.imdecode(np.fromstring(jpg, dtype = np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

        # init values (for x)
        angle_to_turn = 0
        aligned = False

        # gaussian blur to remove noise
        blur = cv2.GaussianBlur(frame, (11, 11), 0)

        # remove everything but specified color
        res, mask = masking(LOWER_LIM, UPPER_LIM, blur)

        # draw references
        draw_static(res)

        # find contours
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
            print("ANGLE_TO_TURN" + str(angle_to_turn))

            # check if shooter is aligned
            aligned = is_aligned(angle_to_turn)
            print("IS_ALIGNED: " + str(aligned))

        # results
        cv2.imshow("NerdyVision", res)
        try:
            # send data to network tables
            table.putNumber('ANGLE_TO_TURN', angle_to_turn)
            table.putBoolean('IS_ALIGNED', aligned)
        except:
            print("DATA NOT SENDING...")


if __name__ == '__main__':
    main()
