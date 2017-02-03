# NerdyVision

### FRC Vision Processing using OpenCV Python

## What You Need 

Python, NumPy and OpenCV

[PyNetworkTables](https://github.com/robotpy/pynetworktables)

Camera (code for Axis M1013 Network Camera or Microsoft Lifecam HD-3000 is included)

[Food](https://www.google.com/#q=food)

## Calibration Mode

1. Set a small rectangle as the calibration box (can change the size in constants)

2. Get frame from camera

3. Find average BGR value inside calibration box

4. Convert BGR value to HSV value

5. Print HSV values for every frame

## Tracking Mode

1. Get frame from camera

2. Remove everything but specified color in frame

3. Find contour for the largest object with the specified color (closest goal)

4. Find centroid of that object (ideal position to shoot)

5. Calculates error and converts from pixels to degrees

6. If the center of the camera is aligned with the center x-axis of the object (goal), then it is ready to shoot

7. Send data over NetworkTables and print in terminal.

## Testing

Testing has been done with 

 - [example images](https://usfirst.collab.net/sf/frs/do/viewRelease/projects.wpilib/frs.sample_programs.2017_c_java_vision_sample?_message=1483834990405) provided by WPILIB (2017 images included in this project under [sample images](https://github.com/tedklin/NerdyVision/tree/master/sample_images))

 - custom made high goal / boiler / gear peg with green highlighter
