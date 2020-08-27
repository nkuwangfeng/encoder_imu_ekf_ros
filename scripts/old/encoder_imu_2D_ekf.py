#!/usr/bin/env python
# HJ: 5/25/20
# This program performs a 2D ekf update using yaw from AHRS

import rospy
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point
import tf
import numpy as np

# record previous state 
ticks_l_prev = 0
tickr_r_prev = 0
first_time = True
initialized_AHRS = False
psi = 0

# measurement update called whenever new encoder data available
def measurement_update():

	global initialized_AHRS
	if not initialized_AHRS:
		return

	# Kalman here
	global state, cov, sigma_yaw, yaw

	# measurement Jacobian
	H = np.reshape(np.array([0,0,1]),(1,3))

	# measurement noise
	Q = sigma_yaw**2

	# Kalman gain. K is 3 by 1
	K = cov.dot(H.T).dot(1.0/(H.dot(cov).dot(H.T)+Q))

	# measurement residual
	z = yaw - state[2]

	# update state
	state = state + np.reshape(K*z,(3))

	# update covariance
	cov = (np.identity(3)-K.dot(H)).dot(cov)

	# publish state
	odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
	odom = Odometry()
	# header
   	odom.header.stamp = rospy.Time.now()
   	odom.header.frame_id = "world"
	# pose
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, state[2])
	odom.pose.pose = Pose(Point(state[0], state[1], 0.0), Quaternion(*odom_quat))

	# publish
	odom_pub.publish(odom)

	# TODO: add covariances in odom

# yaw updated in the background. 'polled'
def callbackAHRSEKF(data):

	global initialized_AHRS
	if not initialized_AHRS:
		initialized_AHRS = True
		print('AHRS Initialized.')

	global roll, pitch, yaw
	angles = tf.transformations.euler_from_quaternion([data.x,data.y,data.z,data.w])

	# store angles in global variables
	roll = angles[0]
	pitch = angles[1]
	yaw = angles[2]

def callbackTicks(data):

	global state, ticks_l_prev, ticks_r_prev, first_time, seq

	# encoder ticks may start as non_zero
	if (first_time):
		ticks_l_prev = data.data[0]
		ticks_r_prev = data.data[1]
		first_time = False		
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
	dpsi = (Dr-Dl)/L
	cpsi = math.cos(state[2]+dpsi/2)
	spsi = math.sin(state[2]+dpsi/2)

	# update state
	state[0] = state[0] + Dc*cpsi # x (m)
	state[1] = state[1] + Dc*spsi # y (m)
	state[2] = state[2] + dpsi # psi (rad)

	# update previous tick count
	ticks_l_prev = ticks_l_curr
	ticks_r_prev = ticks_r_curr

	# update covariance
	global cov, k

	# noise matrix
	U = np.diag([k*np.abs(Dl)**2, k*np.abs(Dr)**2])

	# Jacobian wrt state
	Fx = np.array([[1,0,-Dc*spsi],[0,1,Dc*cpsi],[0,0,1]])

	# Jacobian wrt control input
	Fu = np.array([[0.5*cpsi, 0.5*cpsi],[0.5*spsi, 0.5*spsi],[-1/L, 1/L]])

	cov = Fx.dot(cov).dot(Fx.T) + Fu.dot(U).dot(Fu.T)

	measurement_update()

def initialize():
	global cov
	cov = np.zeros((3,3))

	global k
	# slip constant
	k = 0.1

	global state 
	state = np.array([0,0,0])

	global sigma_yaw 
	sigma_yaw = 0.001

def main():

	# initialize node
    rospy.init_node('ekf_2D', anonymous=True)

    # subscribe to encoders and AHRS
    rospy.Subscriber("wheels", Int32MultiArray, callbackTicks)
    rospy.Subscriber("AHRS_EKF_quat", Quaternion, callbackAHRSEKF)

    initialize()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
