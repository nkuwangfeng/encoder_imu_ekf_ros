# encoder_imu_ekf_ros
ROS package to fuse together IMU (accelerometer + gyroscope) and wheel encoders in an EKF.   
Updates position, velocity, orientation, gyroscope bias and accelerometer bias. Implemented in both C++ and Python. C++ version runs in real time. I wrote this package following standard texts on inertial navigation, taking the data directly from gyroscopes and accelerometers in the EKF state update step. Surprisingly, there aren't many such implementations (including robot_localization). This is probably because the proprietary algorithms onboard IMU's are made by PROs and the attitude from there can be used directly. Also, most people use low grade IMUs that are not capable of integrating the accelerometer in a useful way (high gyro drift). So if your IMU isn't as good as an XSens grade IMU, I don't think this package will work for you.

![GitHub Logo](/results/Motion.png)

Instructions:
1. clone package into catkin_ws/src  
2. catkin_make  
3. roslaunch encoder_imu_ekf_ros cpp_aided_nav.launch  
4. open rviz, create an axis with frame IMU to see the rover driving around.  

External packages needed: eigen

- subscribes to /imu/data for IMU data  
- subscribes to /wheels for encoder data (int64 array of encoder ticks)  

Edit noise parameters in code file as necessary. Will move all parameters to launch file soon. Takes 5 seconds to initialize IMU orientation.

*********NOTE************
More sensors can be added given you can add a subscriber, derive the necessary measurement and noise matrices, and finally call the general 'ekf' function. We will be adding a sun-sensor onto our rover shortly. Theoretical derivation is in Derivation.pdf.

Derivation was done following: Aided Navigation: GPS with High Rate Sensors' by Jay A. Farrell, chapter 10.
