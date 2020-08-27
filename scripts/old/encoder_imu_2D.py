#!/usr/bin/env python
# HJ: 2/11/20
# This program performs dead reckoning using the wheel encoders only

import time
import serial
import rospy
import struct
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu
import tf

# state for 2D robot (x,y,theta)
x = 0
y = 0
theta = 0
seq = 0

# record previous state 
ticks_l_prev = 0
tickr_r_prev = 0
first_time_encoder = True
first_time_imu = True
theta_offset = 0

# previous covariance
# update orientation here. use RK3
def imu_callback(data):
	global theta, theta_offset, first_time_imu
	# get orienttion data
	angles = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w], 'rzyx')
	yaw = angles[0] 
	if (first_time_imu):
		theta_offset = yaw
		first_time_imu = False
		return	
	theta = yaw - theta_offset	
		
def callbackTicks(data):
	global x, y, theta, ticks_l_prev, ticks_r_prev, first_time_encoder, seq
	#rospy.loginfo(rospy.get_caller_id() + "Right %s", data.data)
	if (first_time_encoder):
		ticks_l_prev = data.data[0]
		ticks_r_prev = data.data[1]
		first_time_encoder = False		
		return
		
	# base width (m)
	L = 0.6096
	R = 0.127

	# ticks/m... 1440 ticks per revolution	
	ticks_per_m = 1440/(math.pi*2*R)

	# Distance moved by each wheel
	ticks_l_curr = data.data[0]
	ticks_r_curr = data.data[1]

	# Compute distance moved by each wheel	
	Dl = (ticks_l_curr-ticks_l_prev)/ticks_per_m
	Dr = (ticks_r_curr-ticks_r_prev)/ticks_per_m
	Dc = (Dl+Dr)/2
	
	# update states
	x = x + Dc*math.cos(theta)
	y = y + Dc*math.sin(theta)
	# theta = theta + (Dr-Dl)/L

	# update previous tick count
	ticks_l_prev = ticks_l_curr
	ticks_r_prev = ticks_r_curr
		
	# odometry publisher
	odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
	odom = Odometry()

	# header
	odom.header.seq = seq
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "map"

	# pose
	# quaternion created from yaw
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
	odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

	odom_pub.publish(odom)

	seq += 1


def main():
	# start node
	rospy.init_node('dead_reckoning', anonymous=True)

	# subscribe to encoder
	rospy.Subscriber("wheels", Int32MultiArray, callbackTicks)

	# subscribe to IMU
	rospy.Subscriber("/imu/data", Imu, imu_callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
