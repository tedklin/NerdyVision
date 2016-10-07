# author: tedfoodlin
# FRC Vision testing with OpenCV
# Uses a parallelogram contour

import cv2
import numpy as np
import sys

# for use with OSX
sys.path.append('/usr/local/lib/python2.7/site-packages')
i = 1
j = 1
s = 0

# capture video from camera
cap = cv2.VideoCapture(0)

# lower and upper limits for the green we are looking for
lower_green = np.array([29, 86, 6])
upper_green = np.array([64, 255, 255])

# center of the frame on a mac
frameCenterY = 716/2
frameCenterX = 1278/2

while(True):
    ret, frame = cap.read()

    # remove everything but specified color (green)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    res = cv2.bitwise_and(frame,frame,mask=mask)

    # draw center of camera
    cv2.circle(res, (frameCenterX, frameCenterY), 5, (0, 0, 255), -1)

    # using contours to find the centroid of the green object (goal)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour (closest goal) in the mask
        c = max(cnts, key=cv2.contourArea)
        # draw a rectangle around it
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(res, [box], 0, (0, 0, 255), 2)
        # calculate centroid
        M = cv2.moments(c)
        if M['m00']>0:
            # draw and print center
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            center = (cx, cy)

            cv2.circle(res, center, 5, (0, 0, 255), -1)
            print(center)

            # if the centroid of the green object is within the tolerance zone of the center of the frame
            # it is ready to shoot
            if cx < (frameCenterX+10) and cx > (frameCenterX-10) and cy < (frameCenterY+10) and cy > (frameCenterY-10):
                print("Ready to shoot")
            # otherwise, tell robot to turn left or right and aim the shooter higher or lower
            # to try to get the centroid as close as possible to the center of the frame
            else:
                if cx > frameCenterX:
                    print("Turn Right")
                elif cx < frameCenterX:
                    print("Turn Left")

                if cy > frameCenterY:
                    print("Aim Lower")
                elif cy < frameCenterY:
                    print("Aim Higher")

        cv2.imshow("res", res)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()