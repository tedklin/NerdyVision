# NerdyVision

### FRC Vision testing using OpenCV Python

## Author 
tedfoodlin

## What You Need 

Python, NumPy and OpenCV (pyimagesearch has some nice tutorials on installing OpenCV Python)

[PyNetworkTables](https://github.com/robotpy/pynetworktables)

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

5. If the center of the camera is aligned with the center x-axis of the object (goal), then it is ready to shoot

6. Otherwise, calculates left and right motor values (drivetrain) in order to get align the center x-axis of the camera and the center x-axis of the object. Outputs to SmartDashboard through network tables and prints info to terminal.
