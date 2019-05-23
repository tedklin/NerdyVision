import numpy as np

"""Constants"""
__author__ = "tedlin"

# HSV range values for testing sample images
SAMPLE_LOWER = np.array([80, 70, 80])
SAMPLE_UPPER = np.array([100, 300, 300])

# red for real thing
# LOWER_LIMIT = np.array([0, 79, 50])
# UPPER_LIMIT = np.array([10, 255, 255])

# green for tests
LOWER_LIMIT = np.array([60, 79, 50])
UPPER_LIMIT = np.array([80, 255, 255])

# Dimensions in use (some really sketch waterproof camera)
FRAME_X = 320
FRAME_Y = 240
FOV_ANGLE = 59.02039664
DEGREES_PER_PIXEL = FOV_ANGLE / FRAME_X
FRAME_CX = int(FRAME_X/2)
FRAME_CY = int(FRAME_Y/2)
CAMERA_VERTICAL_ANGLE = 0 # the angle of camera from x plane
TARGET_HEIGHT_DIFFERENCE = 0 # the difference in height (inches) between target and camera on robot

# Mac webcam dimensions (approx)
MAC_FRAME_X = 1280
MAC_FRAME_Y = 720
MAC_FOV_ANGLE = 60
MAC_FOCAL_LENGTH = 15.118110236

# High goal dimensions
MIN_GOAL_AREA = 1936
MAX_GOAL_AREA = 30000
