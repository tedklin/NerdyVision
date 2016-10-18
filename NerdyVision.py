import cv2
import numpy as np
import math
import sys

"""FRC Vision testing with OpenCV Python"""
__author__ = "tedfoodlin"


# For use with OSX and virtualenv
sys.path.append('/usr/local/lib/python2.7/site-packages')

# Capture video from camera
cap = cv2.VideoCapture(0)


# Set modes (if you don't want user input)
CAL_MODE_ON = False
TRACK_MODE_ON = True

# HSV range values for different colors
LOWER_GREEN = np.array([30, 60, 60])
UPPER_GREEN = np.array([70, 255, 255])
LOWER_PINK = np.array([150, 60, 60])
UPPER_PINK = np.array([170, 255, 255])

# Set HSV range
LOWER_LIM = LOWER_GREEN
UPPER_LIM = UPPER_GREEN

# Frame dimensions
FRAME_Y = 716
FRAME_X = 1278
FRAME_CENTER_Y = FRAME_Y / 2
FRAME_CENTER_X = FRAME_X / 2

# Calibration box dimensions
CAL_AREA = 1600
CAL_SIZE = int(math.sqrt(CAL_AREA))
CAL_UP = FRAME_CENTER_Y + (CAL_SIZE / 2)
CAL_LO = FRAME_CENTER_Y - (CAL_SIZE / 2)
CAL_R = FRAME_CENTER_X - (CAL_SIZE / 2)
CAL_L = FRAME_CENTER_X + (CAL_SIZE / 2)
CAL_UL = (CAL_L, CAL_UP)
CAL_LR = (CAL_R, CAL_LO)


def check_modes():
    """Check which modes are on based on user input."""
    if raw_input("Calibration mode on? (y/n)") == "y":
        cal = True
    else:
        cal = False
    if raw_input("Tracking mode on? (y/n)") == "y":
        track = True
    else:
        track = False
    return cal, track


def calibration_box(img):
    """Return HSV color in the calibration box."""
    cv2.rectangle(img, CAL_UL, CAL_LR, (0, 255, 0), thickness=1)
    roi = img[CAL_LO:CAL_UP, CAL_R:CAL_L]
    average_color_per_row = np.average(roi, axis=0)
    average_color = np.average(average_color_per_row, axis=0)
    average_color = np.uint8([[average_color]])
    hsv = cv2.cvtColor(average_color, cv2.COLOR_BGR2HSV)
    return hsv


def masking(lower, upper, frame):
    """Mask for specified color ranges."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res, mask


def draw_static(img):
    """Draw references on frame."""
    # draw center of frame
    cv2.circle(img, (FRAME_CENTER_X, FRAME_CENTER_Y), 5,
               (0, 0, 255), -1)
    # draw reference line for x position
    cv2.line(img, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, FRAME_Y),
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


def report_command(cx):
    """Report robot commands to terminal."""
    # if it is aligned with the center y-axis, it is ready to shoot
    if FRAME_CENTER_X + 10 > cx > FRAME_CENTER_X - 10:
        print("X Aligned")
    # otherwise, tell robot to turn left or right to align with goal
    else:
        if cx > FRAME_CENTER_X + 10:
            print("Turn Right")
        elif cx < FRAME_CENTER_X - 10:
            print("Turn Left")


def report_y(cy):
    """Report state of y to terminal"""
    if FRAME_CENTER_Y + 10 > cy > FRAME_CENTER_Y - 10:
        print("Y Aligned")
    else:
        if cy > FRAME_CENTER_Y + 10:
            print("Aim Lower")
        elif cy < FRAME_CENTER_Y - 10:
            print("Aim Higher")


def main():
    # set modes (default without user input)
    cal_mode_on = CAL_MODE_ON
    track_mode_on = TRACK_MODE_ON

    # turn on modes specified by user
    # comment out next line if this feature is not desired
    cal_mode_on, track_mode_on = check_modes()

    while True:
        ret, frame = cap.read()

        # calibration
        if cal_mode_on:
            print(np.array_str(calibration_box(frame)))
            cv2.imshow("NerdyCalibration", frame)

        # tracking
        if track_mode_on:
            # init states (for x)
            turn_right = False
            turn_left = False
            ready = False

            # remove everything but specified color
            res, mask = masking(LOWER_LIM, UPPER_LIM, frame)

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

                            # draw and print center
                            cv2.circle(res, center, 5, (255, 0, 0), -1)
                            print(center)

                            # if it is aligned with the center y-axis
                            # it is ready to shoot
                            if FRAME_CENTER_X + 10 > cx > FRAME_CENTER_X - 10:
                                ready = True
                            # otherwise, tell robot to turn to align
                            else:
                                if cx > FRAME_CENTER_X + 10:
                                    turn_right = True
                                elif cx < FRAME_CENTER_X - 10:
                                    turn_left = True

                            # report the commands given to robot on terminal
                            report_command(cx)
                            # report state of y (useful but not necessary)
                            report_y(cy)
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
