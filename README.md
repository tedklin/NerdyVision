# NerdyVision

###FRC Vision testing using OpenCV Python

##Author 
tedfoodlin

##Calibration mode

1. Set a small rectangle as the calibration box (can change the size in constants)

2. Get frame from camera

3. Find average BGR value inside calibration box

4. Convert BGR value to HSV value

5. Print HSV values for every frame

##Tracking mode

1. Get frame from camera

2. Remove everything but specified color in frame

3. Find contour for the largest object with the specified color (closest goal)

4. Find centroid of that object (ideal position to shoot)

5. If the center of the camera is aligned with the center x-axis of the object (goal), then it is ready to shoot

6. Otherwise, outputs how to move in order to get align the center x-axis of the camera and the center x-axis of the object.
