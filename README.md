# encoder_imu_ekf_ros
ROS package to fuse together IMU (accelerometer + gyroscope) and wheel encoders in an EKF. Updates position, velocity, orientation, gyroscope bias and accelerometer bias. Implemented in both C++ and Python. C++ version runs in real time. I wrote this package because the standard robot_localization package doesn't work well and in the way standard texts on inertial navigation describe how state estimation with IMUs should operate. Also, no other available software package seemed as flexible in development. 

![GitHub Logo](/results/Motion.png)

Clone into your catkin worskpace.  
catkin_make  
roslaunch encoder_imu_ekf_ros cpp_aided_nav.launch  
subscribes to /imu/data for IMU data  
subscribes to /wheels for encoder data (int64 array of encoder ticks)  
open rviz, create an axis with frame IMU to see the rover driving around.  

Edit parameters in code file as necessary. Will move all parameters to launch file soon. Takes 5 seconds to initialize IMU orientation.

*********NOTE************
More sensors can be added given you can add a subscriber, derive the necessary measurement and noise matrices, and finally call the general 'ekf' function. We will be adding a sun-sensor onto our rover shortly.  

Derivation was done following: Aided Navigation: GPS with High Rate Sensors' by Jay A. Farrell, chapter 10.
