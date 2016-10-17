# NerdyVision

###FRC Vision testing using OpenCV Python

##Author 
tedfoodlin

##Calibration mode

1. Sets a small rectangle as the calibration box (can change the size in constants)

2. Gets frame from camera

3. Finds average BGR value inside calibration box

4. Converts BGR value to HSV value

5. Prints HSV values for every frame

##Tracking mode

1. Gets frame from camera

2. Removes everything but specified color in frame

3. Finds contour for the largest object with the specified color (closest goal)

4. Finds centroid of that object (ideal position to shoot)

5. If the centroid of the object is already within the small specified tolerance zone around the center of the camera, outputs "ready to shoot" (assuming that the camera is at the center of the shooter, otherwise, adjust the frameCenterX and frameCenterY according to position of camera relative to the shooter)

6. Otherwise, outputs how to move in order to get the centroid of the object as close as possible to the center of the camara 
