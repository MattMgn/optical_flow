# optical_flow
This ROS package aimed to provide odometry from raspberry camera mounted downward on quadcopter. It relies on Lucas Kanade algorithm contained in opencv

Optical flow is computed from raw image if only compressed image is published from camera, use republish from image_transport package as shown in 'optical_flow_rosbag.launch'

### TODO  :
add rangefinder information for depth estimation and velocity calculation
add estimator (kalman) with constant acceleration model for computing odometry
