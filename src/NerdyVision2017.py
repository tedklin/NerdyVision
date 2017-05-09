import logging
import os
import cv2
from CameraStream import CameraStream
from networktables import NetworkTable
import NerdyConstants
import NerdyFunctions
logging.basicConfig(level=logging.DEBUG)

"""2017 FRC Vision Processing on Raspberry Pi with Microsoft Lifecam"""
__author__ = "tedfoodlin"

if not os.path.isdir("/tmp/stream"):
   os.makedirs("/tmp/stream")

#cap = CameraStream(src=-1).start()
cap = cv2.VideoCapture(-1)
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

    NetworkTable.setIPAddress("roboRIO-687-FRC.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    SmartDashboard = NetworkTable.getTable("NerdyVision")
    print("NetworkTables initialized")

    while 687:
        angle_to_turn = 0
        aligned = False

        ret, frame = cap.read()
        blur = cv2.GaussianBlur(frame, (11, 11), 0)
        # kernel = np.ones((5, 5), np.uint8)
        # erosion = cv2.erode(frame, kernel, iterations=1)
        # dilation = cv2.dilate(erosion, kernel, iterations=1)
        res, mask = NerdyFunctions.mask(NerdyConstants.LOWER_GREEN, NerdyConstants.UPPER_GREEN, blur)

        _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None

        if len(cnts) > 1:
            centers_x = [0]
            centers_y = [0]

            for i in range(len(cnts)):
                c = cnts[i]
                area = cv2.contourArea(c)
                if NerdyConstants.MIN_GEAR_AREA < area < NerdyConstants.MAX_GEAR_AREA:
                    goal = NerdyFunctions.polygon(c, 0.02)

                    cv2.drawContours(res, [goal], 0, (255, 0, 0), 5)

                    M = cv2.moments(goal)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        center = (cx, cy)

                        cv2.circle(res, center, 5, (255, 0, 0), -1)

                        centers_x.append(cx)
                        centers_y.append(cy)

            if len(centers_x) == 3 and len(centers_y) == 3:
                target_x = (centers_x[1] + centers_x[2])/2
                target_y = (centers_y[1] + centers_y[2])/2
                target = (target_x, target_y)
                cv2.circle(res, target, 5, (0, 255, 0), -1)
                print(target_x)
                print(target_y)

                error = target_x - NerdyConstants.FRAME_CX
                angle_to_turn = NerdyFunctions.calc_horiz_angle(error)
                print("ANGLE_TO_TURN: " + str(angle_to_turn))
                aligned = NerdyFunctions.is_aligned(angle_to_turn)
                print("IS_ALIGNED: " + str(aligned))

        NerdyFunctions.draw_static(res)
        cv2.imshow("NerdyVision", res)
        try:
            SmartDashboard.putNumber('ANGLE_TO_TURN', angle_to_turn)
            SmartDashboard.putBoolean('IS_ALIGNED', aligned)
        except:
            print("DATA NOT SENDING...")

        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
