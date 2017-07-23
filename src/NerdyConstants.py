import numpy as np

"""Important Constants"""
__author__ = "tedfoodlin"

# HSV range values for testing sample images
SAMPLE_LOWER = np.array([80, 70, 80])
SAMPLE_UPPER = np.array([100, 300, 300])

# HSV range values for green
LOWER_GREEN = np.array([50, 50, 50])
UPPER_GREEN = np.array([70, 255, 255])

# Dimensions in use (Microsoft Lifecam HD-3000)
FRAME_X = 320
FRAME_Y = 240
FOV_ANGLE = 59.02039664
DEGREES_PER_PIXEL = FOV_ANGLE / FRAME_X
FRAME_CX = int(FRAME_X/2)
FRAME_CY = int(FRAME_Y/2)

# Mac webcam dimensions (approx)
MAC_FRAME_X = 1280
MAC_FRAME_Y = 720
MAC_FOV_ANGLE = 60
MAC_FOCAL_LENGTH = 15.118110236

# Gear dimensions
MIN_GEAR_AREA = 15000
MAX_GEAR_AREA = 50000
MIN_GEAR_ASPECT = 0.2
MAX_GEAR_ASPECT = 0.6

# High goal dimensions
MIN_GOAL_AREA = 0

# Boiler dimensions
MIN_BOILER_AREA = 300
