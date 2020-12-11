# HandEye-Calibration
Finding the transformation from gripper to camera in a UR10e robot.
This project finds the solution for equation AX=XB or saying different it finds the transformation from gripper to camera in a Robot UR10e with a 2D camera called WENGLOR B50M100.
calibrFunction.m make the calibration of the camera using 20 images from diiferent angles using a checkerboard.
extrinsicFunc.m is another function  that calculates the rotation and translation of the camera .
mainFunc.m is the main part of the project .Here we find two transformations that are called Tc(transformation from camera to world coordinate system) and
Tg(transformation from gripper to camera ).Then we calculate X(Hand-Eye) in order to find the thing that we want.
All this procedure it is neccessary for robot to take the object in the ground with and move it to another position automatically.
For more information :armand.palla@stud.unifi.it
