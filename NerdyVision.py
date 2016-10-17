import cv2
import numpy as np
import sys

"""FRC Vision testing with OpenCV"""
__author__ = "tedfoodlin"


# For use with OSX and virtualenv
sys.path.append('/usr/local/lib/python2.7/site-packages')

# Capture video from camera
cap = cv2.VideoCapture(0)


# ---------------- CONSTANTS ---------------- #
# HSV values
LOWER_GREEN = np.array([50, 20, 20])
UPPER_GREEN = np.array([70, 255, 255])
LOWER_PINK = np.array([150, 60, 60])
UPPER_PINK = np.array([170, 255, 255])

# Frame dimensions
FRAME_Y = 716
FRAME_X = 1278
FRAME_CENTER_Y = FRAME_Y / 2
FRAME_CENTER_X = FRAME_X / 2
# ------------------------------------------- #


# ---------------- FUNCTIONS ---------------- #
def masking(lower, upper, frame):
    """Mask for specified color ranges."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask


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


def report_command(cx):
    """Report robot commands to terminal."""
    # if it is aligned with the center y-axis, it is ready to shoot
    if cx < (FRAME_CENTER_X + 10) and cx > (FRAME_CENTER_X - 10):
        print("X Aligned")
    # otherwise, tell robot to turn left or right to align with goal
    else:
        if cx > FRAME_CENTER_X + 10:
            print("Turn Right")
        elif cx < FRAME_CENTER_X - 10:
            print("Turn Left")


def report_y(cy):
    """Report state of y to terminal"""
    if cy < (FRAME_CENTER_Y + 10) and cy > (FRAME_CENTER_Y - 10):
        print("Y Aligned")
    else:
        if cy > FRAME_CENTER_Y + 10:
            print("Aim Lower")
        elif cy < FRAME_CENTER_Y - 10:
            print("Aim Higher")
# ------------------------------------------- #


def main():
    # iterative tracking
    while 687:
        # init states (for x only)
        turnRight = False
        turnLeft = False
        ready = False

        ret, frame = cap.read()

        # remove everything but specified color
        res, mask = masking(LOWER_GREEN, UPPER_GREEN, frame)

        # draw center of camera
        cv2.circle(res, (frameCenterX, frameCenterY), 5, (0, 0, 255), -1)
        # draw line for x position (we have set shooting angles for y)
        cv2.line(res, (frameCenterX, 0), (frameCenterX, y), (0, 0, 255), 2)

        # find contour of goal
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour (closest goal) in the mask
            c = max(cnts, key=cv2.contourArea)

            # make sure the largest contour is large enough to be significant
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

                        # draw and print center
                        cv2.circle(res, center, 5, (255, 0, 0), -1)
                        print(center)

                        # define ranges of tolerance
                        upper_range = frameCenterX + 10
                        lower_range = frameCenterX - 10
                        # if it is aligned with the center y-axis
                        # it is ready to shoot
                        if cx < (upper_range) and cx > (lower_range):
                            ready = True
                        # otherwise, tell robot to turn left or right to align
                        else:
                            if cx > upper_range:
                                turnRight = True
                            elif cx < lower_range:
                                turnLeft = True

                        # report the commands given to robot on terminal
                        report_command(cx)
                        # report state of y (useful but not necessary)
                        report_y(cy)
                else:
                    print("Goal contour not found")
            else:
                print("Goal contour not found")

        # show results
        cv2.imshow("NerdyVision", res)

        # capture a keypress.
        key = cv2.waitKey(10) & 0xFF
        # escape key.
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
