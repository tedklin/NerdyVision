import logging
import cv2
import time
from networktables import NetworkTable
import NerdyConstants
import NerdyFunctions
logging.basicConfig(level=logging.DEBUG)

"""2017 FRC Vision testing on laptop with Microsoft Lifecam"""
__author__ = "tedfoodlin"

# Capture video from camera (0 for laptop webcam, 1 for USB camera)
cap = cv2.VideoCapture(-1)

# for sample image testing (images from 1 to 32)
sample_image = 32

# default modes
shooting = False
gears = True


def check_modes():
    """Check which modes are on based on user input."""
    shooting = False
    gears = False
    if raw_input("Shooting mode on? (y/n)") == "y":
        shooting = True
    if raw_input("Gears mode on? (y/n)") == "y":
        gears = True
    return shooting, gears


def main():
    # turn on modes specified by user
    # uncomment next line if this feature is desired
    # shooting, gears = check_modes()

    # network table setup
    NetworkTable.setIPAddress("roboRIO-687-FRC.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    SmartDashboard = NetworkTable.getTable("NerdyVision")
    print("NetworkTables initialized")

    # adjust camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, NerdyConstants.FRAME_X)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, NerdyConstants.FRAME_Y)
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
        res, mask = NerdyFunctions.mask(NerdyConstants.LOWER_GREEN, NerdyConstants.UPPER_GREEN, blur)

        # draw references
        NerdyFunctions.draw_static(res)

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

                if area > NerdyConstants.MIN_BOILER_AREA:
                    goal = NerdyFunctions.polygon(c, 0)

                    # draw the contour
                    cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)

                    # calculate centroid
                    M = cv2.moments(goal)
                    if M['m00'] > 0:
                        cx, cy = NerdyFunctions.calc_center(M)
                        center = (cx, cy)

                        # draw centroid
                        cv2.circle(res, center, 5, (255, 0, 0), -1)

                        # calculate error in degrees
                        error = cx - NerdyFunctions.FRAME_CX
                        angle_to_turn = NerdyFunctions.calc_horiz_angle(error)
                        print("ANGLE TO TURN " + str(angle_to_turn))

                        # check if shooter is aligned
                        aligned = NerdyFunctions.is_aligned(angle_to_turn)
                        print("ALIGNED " + str(aligned))

                        NerdyFunctions.report_command(error)
                        NerdyFunctions.report_y(cy)

        elif gears:
            # only proceed if at least two contours (two blocks around peg) was found
            if len(cnts) > 1:
                centers_x = [0]
                centers_y = [0]

                # find the two blocks in the mask based on areas
                for i in range(len(cnts)):
                    c = cnts[i]
                    area = cv2.contourArea(c)
                    if NerdyConstants.MIN_GEAR_AREA < area < NerdyConstants.MAX_GEAR_AREA:
                        goal = NerdyFunctions.polygon(c, 0.02)

                        # draw the contour
                        cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)

                        M = cv2.moments(goal)
                        if M['m00'] > 0:
                            cx, cy = NerdyFunctions.calc_center(M)
                            center = (cx, cy)

                            # draw centroid
                            cv2.circle(res, center, 5, (255, 0, 0), -1)

                            centers_x.append(cx)
                            centers_y.append(cy)

                # calculate center of two contours (blocks next to peg)
                if len(centers_x) == 3 and len(centers_y) == 3:
                    target_x = NerdyFunctions.avg(centers_x[1], centers_x[2])
                    target_y = NerdyFunctions.avg(centers_y[1], centers_y[2])
                    target = (target_x, target_y)
                    cv2.circle(res, target, 5, (0, 255, 0), -1)
                    print(target_x)
                    print(target_y)

                    # calculate angle to turn
                    error = target_x - NerdyConstants.FRAME_CX
                    angle_to_turn = NerdyFunctions.calc_horiz_angle(error)
                    print("ANGLE TO TURN " + str(angle_to_turn))

                    # check if gear mechanism is aligned
                    aligned = NerdyFunctions.is_aligned(angle_to_turn)
                    print("ALIGNED " + str(aligned))

                    NerdyFunctions.report_command(error)

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
