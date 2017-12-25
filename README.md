# NerdyVision

### FRC Vision Processing using OpenCV Python

## What You Need 

Python 2.x or 3.x, NumPy and OpenCV

[PyNetworkTables](https://github.com/robotpy/pynetworktables)

[MJPG streamer](https://sourceforge.net/projects/mjpg-streamer/)

Raspberry Pi compatible USB camera (Microsoft Lifecam HD-3000 used in this project)

[Food](https://www.google.com/#q=food)

## Calibration Mode 1

1. Set a small centered rectangle as the calibration box (can change the size)

2. Capture frame from camera

3. Find average BGR value inside calibration box

4. Convert BGR value to HSV value

5. Print average HSV value inside calibration box

## Calibration Mode 2

1. Create adjustable trackbars for minimum and maximum H, S, and V values

2. Get HSV range from trackbars

3. Capture frame from camera

4. Apply HSV mask 

5. Print HSV values in use

## Calibration Mode 3

1. Create adjustable trackbars for minimum and maximum sqrt(area) values

2. Get area values (square the values from trackbars)

3. Capture frame from camera

4. Apply HSV mask

5. Find and draw contours within area constraints

6. Print area values in use

## Tracking Mode

1. Capture frame from camera

2. Remove everything but specified color in frame

3. Find contour for largest object with the specified color within area constraints

4. Find centroid of that object (ideal position to shoot / drop off gear)

5. Calculate error (pixels) of centroid of object from center of frame

6. Convert error in pixels to real world units

7. Send data over NetworkTables and print in terminal

## Testing

Testing has been done with 

 - [example images](https://usfirst.collab.net/sf/frs/do/viewRelease/projects.wpilib/frs.sample_programs.2017_c_java_vision_sample?_message=1483834990405) provided by WPILIB (2017 images included in this project under [sample images](https://github.com/tedklin/NerdyVision/tree/master/sample_images))

    - Tests for all example images in the [boiler folder](https://github.com/tedklin/NerdyVision/tree/master/sample_images/LED_Boiler) successful except for images 7 and 32

 - custom made high goal / boiler / gear peg with green highlighter / marker
