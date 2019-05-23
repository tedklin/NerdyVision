import cv2
import NerdyConstants
import math

"""FRC Image Processing Functions"""
__author__ = "tedlin"


def mask(lower, upper, frame):
    """Mask for specified color ranges."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask


def draw_static(img):
    """Draw references on frame."""
    # cv2.line(img, (NerdyConstants.FRAME_CX, int(0.25 * NerdyConstants.FRAME_Y)),
    #          (NerdyConstants.FRAME_CX, int(0.75 * NerdyConstants.FRAME_Y)), (255, 0, 0), 3)
    cv2.line(img, (int(0.25 * NerdyConstants.FRAME_X), NerdyConstants.FRAME_CY),
             (int(0.75 * NerdyConstants.FRAME_X), NerdyConstants.FRAME_CY), (255, 0, 0), 3)


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
    """Calculate the horizontal angle from pixel error."""
    #return math.atan(error / NerdyConstants.FOCAL_LENGTH_X)
    return error * NerdyConstants.DEGREES_PER_PIXEL


def calc_vertical_angle(error):
    """Calculate the vertical angle from pixel error"""
    #return math.atan(error / NerdyConstants.FOCAL_LENGTH_Y)
    return (error * NerdyConstants.DEGREES_PER_PIXEL) + NerdyConstants.CAMERA_VERTICAL_ANGLE


def calc_distance(error):
    """Calculate the distance from target from pixel error"""
    if error != 0:
        return NerdyConstants.TARGET_HEIGHT_DIFFERENCE / math.tan(math.radians(calc_vertical_angle(error)))
    else:
        return 0


def avg(x1, x2):
    """"Take average of 2 numbers."""
    sum = x1 + x2
    return sum / 2


def is_aligned(error):
    """Check alignment."""
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
    """Testing - report state of y to terminal."""
    if NerdyConstants.FRAME_CY + 10 > cy > NerdyConstants.FRAME_CY - 10:
        print("Y Aligned")
    else:
        if cy > NerdyConstants.FRAME_CY + 10:
            print("Aim Lower")
        elif cy < NerdyConstants.FRAME_CY - 10:
            print("Aim Higher")
