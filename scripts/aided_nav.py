#!/usr/bin/env python
# HJ: 5/28/20

import rospy
from sensor_msgs.msg import Imu
from imu_ekf_ros.srv import *

# other
import numpy as np
import math
from std_msgs.msg import Int32MultiArray
from pyquaternion import Quaternion
from scipy.linalg import expm
from helper import to_skew

#debugging
import tf

# accelerometer
rover_stationary = False # whether or not to do accelerometer corrections
accel_counter = 0
g_pred_sum = 0

# encoder
first_time_enc = True # for offsets
ticks_l_prev = 0
tickr_r_prev = 0
counter = 0

def measurement_update():

	# encoder state (acts like a sensor measurement)
	global enc_state

	# predicted position from encoder model
	p_meas = enc_state[0]
	yaw_meas = enc_state[1]

	# encoder measurement matrix: He is 4 x 15
	He = np.hstack((np.identity(3),np.zeros((3,12))))
	He = np.vstack((He, np.concatenate((np.zeros(8),np.array([1]),np.zeros(6)))))

	# encoder noise matrix
	global enc_Re 
	Re = enc_Re

	# access current predicted state
	global state
	p = state[0]
	b = state[2]
	x_a = state[3]
	x_g = state[4]

	# to pyQuaternion format
	b_q = Quaternion(array=b)
	b_inv = b_q.inverse # body to nav
	R_body_to_nav = b_inv.rotation_matrix

	angles = tf.transformations.euler_from_quaternion([b_inv[1],b_inv[2],b_inv[3],b_inv[0]],'rzyx')
	
	# predicted yaw
	yaw_pred = angles[0]
	
	# predicted position
	p_pred = p
	
	# if robot stationary
	global rover_stationary
	rover_stationary = False
	# use accelerometer data if rover stationary
	if 	rover_stationary:

		# measurement matrix accelerometer: Ha is 2 x 15
		global g
		#Ha = np.array([[0,g,0],[-g, 0,0],[0,0,0]]) 	
		#Ha = np.hstack((np.zeros((3,6)),Ha,R_body_to_nav,np.zeros((3,3)))) 

		Ha = np.array([[0,g,0],[-g, 0,0]]) # ERROR HERE. SIGN PROBLEM
		Ha = np.hstack((np.zeros((2,6)),Ha,R_body_to_nav[:2,:],np.zeros((2,3)))) 

		# combined measurement matrices
		H = np.vstack((He,Ha))

		# get accelerometer noise matrix
		global Ra

		# combined noise matrix: R is 5 x 5
		R = np.diag(np.concatenate((np.diagonal(Re),np.diagonal(Ra))))

		# predicted gravity vector
		global g_pred 

		# stack predicted 'measurement': predicted position and gravity vector
		#y_pred = np.concatenate((p_pred,np.array([yaw_pred]),g_pred))
		y_pred = np.concatenate((p_pred,np.array([yaw_pred]),g_pred[:2]))

		# known gravity acceleration
		# g_vec = np.array([0,0,g]) 
		g_vec = np.array([0,0]) 

		# actual measurement is output position of encoder model and known gravity acceleration
		y = np.concatenate((p_meas,np.array([yaw_meas]),g_vec))

		# restart counter # should it be somewhere else
		rover_stationary = False

		print('Stationary update')

	else:

		# measurement and noise matrices depend on encoder only
		H = He
		R = Re

		# predicted measurement is position and yaw
		y_pred = np.append(p_pred,yaw_pred)

		# actual measurement is output position and yaw of encoder model
		y = np.append(p_meas,yaw_meas)	


	# previous covariance: 15x15
	global cov

	# compute Kalman gain
	try:
		K = cov.dot(H.T).dot(np.linalg.inv(R+H.dot(cov).dot(H.T)))
	except:
		print('Kalman gain error')
		return

	# residual z
	z = y-y_pred

	# angles can be close to pi and -pi
	if abs(z[3]) > math.pi:
		z[3] -= np.sign(z[3])*2*math.pi

	# compute state correction
	dx = K.dot(z)

	# rotation update 
	ro = dx[6:9]
	P = to_skew(ro)
	R_nav_to_body = R_body_to_nav.T
	R_nav_to_body_next = R_nav_to_body.dot(np.identity(3) - P) # (10.67)

	# reorthogonalize matrix... required by PyQuaternion... use SVD
	U, S, Vh = np.linalg.svd(R_nav_to_body_next)
	R_nav_to_body_next = U.dot(Vh)

	# compute next quaternion
	b_next = Quaternion(matrix=R_nav_to_body_next).elements

	# # update state 
	if np.linalg.norm(dx[:3]) > 0.1:
		print('pos update crazy')
	# print(ro)
	# 	print(p_pred)
	# 	print(p_meas)
	# 	print()
	if np.linalg.norm(dx[3:6]) > 1:
		print('vel update crazy')

	# update state
	state[0] = state[0] + dx[:3]
	state[1] = state[1] + dx[3:6]
	state[2] = b_next # update quaternion: [w, x, y, z]
	state[3] = state[3] + dx[9:12] # update accel bias
	state[4] = state[4] + dx[12:]  # update gyro bias

	# update covariance
	cov = (np.identity(15)-K.dot(H)).dot(cov)

	# update encoder start state: position and orientation
	enc_state[0] = state[0]
	enc_state[2] = b_next

	global counter
	if counter % 20 == 0:
		print(state[0])
	
	counter += 1


def encoders_callback(data):

	# encoder ticks may start as non_zero. record offsets
	global first_time_enc, ticks_l_prev, ticks_r_prev
	if (first_time_enc):
		ticks_l_prev = data.data[0]
		ticks_r_prev = data.data[1]
		first_time_enc = False	
		return
	
	# TODO: put in launch file
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
	Dc = (Dl+Dr)/2.0

	# Store previous set of readings
	ticks_l_prev = data.data[0]
	ticks_r_prev = data.data[1]

	# get current position estimate. starts at end of last measurement update
	global enc_state
	p = enc_state[0]
	x = p[0]
	y = p[1]
	z = p[2]

	# orientation
	b = enc_state[2]
	b_q = Quaternion(array=b)

	# get current yaw pitch roll. tf: Quaternions ix+jy+kz+w are represented as [x, y, z, w].
	b_inv = b_q.inverse #
	angles = tf.transformations.euler_from_quaternion([b_inv[1],b_inv[2],b_inv[3],b_inv[0]],'rzyx')

	# store angles in global variables
	phi = angles[2] # roll
	theta = angles[1] # pitch
	psi = angles[0] # yaw
 
	# kinematic model
	x = x + Dc*math.cos(psi)*math.cos(theta)
	y = y + Dc*math.sin(psi)*math.cos(theta)
	z = z - Dc*math.sin(theta)
	yaw = psi + 1/L*(Dr-Dl)*math.cos(phi)/math.cos(theta)

	# store in state 
	enc_state[0] = np.array([x,y,z])
	enc_state[1] = yaw

	# encoder covariance
	global enc_Re, k
	cpsi = math.cos(psi)
	spsi = math.sin(psi)
	ctheta = math.cos(theta)
	stheta = math.sin(theta)
	cphi = math.cos(phi)

	# variance = k*Dl
	U = np.diag([(k*np.abs(Dl))**2,(k*np.abs(Dr))**2])
	
	# with yaw
	Fu = np.array([[0.5*cpsi*ctheta, 0.5*cpsi*ctheta],[0.5*psi*ctheta,0.5*psi*ctheta],[-0.5*stheta, -0.5*stheta],[-1/L*cphi/ctheta,1/L*cphi/ctheta]])
	
	# store encoder noise in global variable
	enc_Re = Fu.dot(U).dot(Fu.T)

	measurement_update()


# update orientation here. use RK3
def imu_callback(data):

	# sample time 
	global dt
	dt = 1/200.0

	# get gyro data
	w = np.array([data.angular_velocity.x, data.angular_velocity.y,data.angular_velocity.z])

	# get accelerometer data: specific force in body frame
	f_b = np.array([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])

	### Update state ###

	# get current state
	global state

	# current position
	p = state[0]

	# current velocity
	v = state[1]

	# current orientation
	b = state[2]

	# current accel bias
	x_a = state[3]

	# current gyro bias
	x_g = state[4]

	# subtract out gyroscope bias. w_bn = (-w_nb)
	w = -1*(w-x_g) #
	w_norm = np.linalg.norm(w)

	# subtract out accelerometer bias
	f_b = f_b - x_a

	# differential rotation: [w, x, y, z]
	db = np.concatenate(([math.cos(w_norm*dt/2)],math.sin(w_norm*dt/2)*w/w_norm))

	# convert to pyquaternion format
	b_prev = Quaternion(array=b)
	db = Quaternion(array=db)

	# update orientation
	b_next = db*b_prev

	# get average quaternion by interpolation
	b_avg = Quaternion.slerp(b_prev,b_next,amount=0.5)

	# b is the nav to body transformation. we need body to nav transformation -> invert 
	b_body_to_nav_avg = b_avg.inverse # for specific force (5.9 Principles of GNSS book)

	# rotate specific force into inertial frame
	f_i = b_body_to_nav_avg.rotate(f_b)

	# gravity vector
	global g
	g_vec = np.array([0,0,g])

	# get acceleration in inertial frame. (acceleration of body wrt inertial frame in inertial frame)
	a_i = f_i + g_vec

	# update position (5.16 Principles of GNSS book)
	p = p + v*dt + 0.5*a_i*dt**2

	# DEBUG:
	if np.linalg.norm(v*dt + 0.5*a_i*dt**2) > 0.1:
		print("IMU update crazy")
		print(state)
	# 	print(np.linalg.norm(v*dt + 0.5*a_i*dt**2) )
	# 	print(v)
	# 	print(a_i)
	# 	print(x_a)
	# 	print(x_g)
	# 	print(w)
	# 	print()
	# 	rospy.sleep(100)

	# update velocity
	v = v + a_i*dt

	# store in state -> this is time propagation step. 
	state[0] = p
	state[1] = v
	state[2] = b_next.elements

	### Update covariance ###

	# get rotation matrix corresponding to b_next
	b_body_to_nav = b_next.inverse
	R_body_to_nav = b_body_to_nav.rotation_matrix

	# compute F matrix: F is 15 x 15
	F = np.zeros((15,15))

	# store relevant values in F (see writeup)
	F[:3,3:6] = np.identity(3)
	f_i_skew = to_skew(f_i)
	F[3:6,6:9] = -1*f_i_skew
	F[3:6,9:12] = -1*R_body_to_nav
	F[6:9,12:15] = R_body_to_nav
	F_gg = -1.0/1000*np.identity(3)
	F_aa = -1.0/1000*np.identity(3)
	F[9:12,9:12] = F_aa
	F[12:15,12:15] = F_gg

	# compute system transition matrix Phi
	Phi = expm(F*dt)

	# compute G. G is 15 x 12.
	G = np.zeros((15,12))
	G[3:6,:3] = -R_body_to_nav
	G[6:9,3:6] = R_body_to_nav
	G[9:12,6:9] = np.identity(3)
	G[12:15,9:12] = np.identity(3)

	# get noise matrix
	global Q

	# compute Qdk (discrete noise). Qdk is 15 x 15
	Qdk = G.dot(Q).dot(G.T)*dt

	# get previous covariance
	global cov

	# update covariance (15x15)
	cov = Phi.dot(cov).dot(Phi.T)+Qdk

	### Measurement update stuff for acceleration. Helps correct roll and pitch ###

	# if 50 accelerometer readings were close enough to the gravity vector, robot is stationary
	global accel_counter, rover_stationary, g_pred_sum

	# predict gravity in navigation frame and store prediction in global variable. used in filter.
	num_g_pred = 50
	global g_pred_sum

	# check if current measurement is stationary
	if np.abs(np.linalg.norm(f_b) - g) < 0.03: # tuned. test: you should never be able to hold the imu in your hand and have an update.
		accel_counter += 1
		g_pred_sum += R_body_to_nav.dot(-1*f_b)
	else:
		accel_counter = 0
		g_pred_sum = 0
		rover_stationary = False

	# if 50 consecutive stationary, use accel_data
	if accel_counter == num_g_pred:
		global g_pred
		# averaging
		g_pred = g_pred_sum/num_g_pred
		rover_stationary = True
		accel_counter = 0
		g_pred_sum = 0

	### Visualization ###

	# for visualization: publish tf
	if True:
		b_vis = b_body_to_nav

		# useful for visualization
		br = tf.TransformBroadcaster()

		# send world to IMU transformation
		br.sendTransform((p[0], p[1], p[2]),
		(b_vis[1],b_vis[2],b_vis[3],b_vis[0]),
		rospy.Time.now(),
		"IMU",
		"ENU") # ENU to quat

def initialize():

	print("aided_nav: waiting for initialize_ahrs service")
	# wait for service to become active
	rospy.wait_for_service('initalize_ahrs')

	try:
		# create service method
		initalize_ahrs = rospy.ServiceProxy('initalize_ahrs', initRequest)

		# request the service
		print("aided_nav: Initializing orientation and gyro bias.")
		resp = initalize_ahrs()
		print("aided_nav: Received initalization data.")

		# access service response
		b = resp.init_orientation
		gyro_bias = resp.gyro_bias

		# initialize state: [p, v, b, x_a, x_g] = [position, velocity, quaternion, accel bias, gyro bias],  size 16
		global state
		state = np.array([np.zeros(3),np.zeros(3),np.array([b.w,b.x,b.y,b.z]),np.zeros(3), np.array([gyro_bias[0],gyro_bias[1],gyro_bias[2]])])

		# initialize noise terms
		global dt
		hz = rospy.get_param("imu_hz",200)
		dt = 1.0/hz
		T = 1000.0/hz # number of measurements over rate of IMU
		sigma_xg = rospy.get_param('sigma_xg',0.00000290) # Gyro (rate) random walk
		sigma_nug = rospy.get_param('sigma_nug',0.00068585) # rad/s/rt_Hz, Gyro white noise
		sigma_xa = rospy.get_param('sigma_xa',0.00001483)  # Accel (rate) random walk m/s3 1/sqrt(Hz)
		sigma_nua = rospy.get_param('sigma_nua',0.00220313 ) # accel white noise

		# compute noise matrix for IMU, Q. Q is 12 x 12. 
		global Q
		Q = np.zeros((12,12))
		Q[:3,:3] = sigma_nua**2*np.identity(3)
		Q[3:6,3:6] = sigma_nug**2*np.identity(3)
		Q[6:9,6:9] = sigma_xa**2*np.identity(3)
		Q[9:12,9:12] = sigma_xg**2*np.identity(3)

		# accelerometer noise matrix, used in measurment update. Ra is 3x3
		global Ra
		#Ra = np.identity(3)*sigma_nua**2
		Ra = np.identity(2)*sigma_nua**2
		
		# gravity vector
		global g
		# TODO: put in launch file
		g = 9.8021 # gravity m/s/s

		# initialize covariance: cov is 15 x 15
		global cov 
		cov = np.zeros((15,15))
		cov[6:9,6:9] = np.diag([sigma_nua/g,sigma_nua/g,0])**2/T # Ppp
		cov[9:12,9:12] = np.zeros((3,3)) # Paa
		cov[12:,12:] = np.identity(3)*sigma_nug**2/T # Pgg

		# initialize encoder model state [p,psi,b] = [position, yaw, orientation]
		# encoder acts like a sensor initialized at end of every measurement update
		global enc_state
		enc_state = np.array([np.zeros(3),0,np.array([b.w,b.x,b.y,b.z])])

		# encoder noise model
		global k 
		k = 0.05 # % slip

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def main():

	# initialize node
	rospy.init_node('aided_nav', anonymous=True)

	# initialize parameters
	initialize()

	# subscribe to encoders 
	rospy.Subscriber("wheels", Int32MultiArray, encoders_callback,queue_size = 1)

	# subscribe to IMU
	rospy.Subscriber("/imu/data", Imu, imu_callback, queue_size = 1)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
