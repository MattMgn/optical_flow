# optical_flow
Optical flow to provide odometry from raspberry camera fixed to quadcopter

Optical flow is computed from raw image if  only compressed image is published from camera, use republish from image_transport package as shown in optical_flow.launch

TODO  :
add rangefinder information for depth estimation and velocity calculation
add estimator (kalman) with constant acceleration model for computing odometry
