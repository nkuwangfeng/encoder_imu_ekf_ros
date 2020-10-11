// HJ: August 3, 2020
// EKF adopted from earlier python code, fuses IMU and encoder data 
// ***I didn't use typedefs for clarity for readers
#include "ekf.h"
#include "encoder_imu_ekf_ros/initRequest.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <cstdlib>

void debug(auto str)
{
std::cout << str << std::endl;
}

// convert vector to 3x3 skew symmetric matrix
void to_skew(const Eigen::Matrix<float,3,1> &v, Eigen::Matrix<float,3,3> &m)
{
m << 0, -v(2), v(1),
v(2), 0, -v(0),
-v(1), v(0), 0;
}

// compute state transition matrix
void computePhi(const Eigen::Matrix<float,3,1> &f_i,const Eigen::Matrix<float,3,3> &R_body_to_nav_next, Eigen::Matrix<float,15,15> &Phi)
{
	// compute F matrix: F is 15 x 15
	Eigen::Matrix<float,15,15> F = Eigen::Matrix<float,15,15>::Zero();

	// store relevant values in F (see writeup for derivation) THIS PART IS KEY, where my code differs from standard
	Eigen::Matrix<float,3,3> f_i_skew(3,3);
	to_skew(f_i,f_i_skew);
	F.block<3,3>(0,3) = Eigen::Matrix<float,3,3>::Identity();
	F.block<3,3>(3,6) = -1*f_i_skew; // .block() function was employed as a lvalue
	F.block<3,3>(3,9) = -1*R_body_to_nav_next;
	F.block<3,3>(6,12) = R_body_to_nav_next;
	F.block<3,3>(9,9) = Eigen::Matrix<float,3,3>::Identity()*-1.0/200; // Fa
	F.block<3,3>(12,12) = Eigen::Matrix<float,3,3>::Identity()*-1.0/200; // Fg

	// compute system transition matrix Phi
	Phi = (F*filter.dt).exp();
}

// compute noise matrix
void computeQdk(const Eigen::Matrix<float,3,3> &R_body_to_nav_next, Eigen::Matrix<float,15,15> &Qdk)
{
	// compute G. G is 15 x 12.
	Eigen::Matrix<float,15,12> G = Eigen::Matrix<float,15,12>::Zero();
	G.block<3,3>(3,0) = -1*R_body_to_nav_next;
	G.block<3,3>(6,3) = R_body_to_nav_next;
	G.block<3,3>(9,6) = Eigen::Matrix<float,3,3>::Identity();
	G.block<3,3>(12,9) = Eigen::Matrix<float,3,3>::Identity();
	Qdk = G*(filter.Q)*G.transpose()*filter.dt;
}

void stationaryMeasurementUpdate(const Eigen::Matrix<float,3,3> & R_body_to_nav)
{
	debug("stationary update!");
	rover_stationary = false;

	// stationary update matrix: Ha is 3 x 15
	Eigen::Matrix<float,3,15> Ha = Eigen::Matrix<float,3,15>::Zero();

	// known gravity
	Eigen::Matrix<float,3,1> g_meas(0,0,filter.g);
	Eigen::Matrix<float,3,3> g_skew(3,3);
	to_skew(g_meas, g_skew);
	Ha.block<3,3>(0,6) = -1*g_skew;
	Ha.block<3,3>(0,12) = R_body_to_nav;

	// noise matrix
	Eigen::Matrix<float,3,3> Ra = filter.Ra;

	// variables to fill -> I kept it like this for reviewer clarity. TODO refactor to shorten.
	Eigen::MatrixXf H; // measurement matrix
	Eigen::MatrixXf R; // noise matrix
	Eigen::MatrixXf y_pred; // predicted measurement
	Eigen::MatrixXf y_meas; // actual measurement 

	// measurement and noise matrices
	H = Ha;
	R = Ra;

	y_pred = Eigen::Matrix<float,3,1>::Zero();
	y_meas = Eigen::Matrix<float,3,1>::Zero();

	// predicted measurement is gravity 
	y_pred << g_pred(0), g_pred(1), g_pred(2);

	// actual measurement is actual gravity
	y_meas << g_meas(0), g_meas(1), g_meas(2);

	// predicted gravity: g_pred
	Eigen::Matrix<float,3,1> z = y_meas - y_pred;

	// call filter
	EKF(H.block<2,15>(0,0),R.block<2,2>(0,0),z(Eigen::seq(0,1)), false);
	//EKF(H,R,z, false);

}

void encoderMeasurementUpdate()
{
	// predicted position from encoder model
	Eigen::Matrix<float,3,1> p_meas = enc_meas(Eigen::seq(0,2));

	// encoder measurement matrix: He is 3 x 15
	Eigen::Matrix<float,3,15> He = Eigen::Matrix<float,3,15>::Zero();
	He.setZero(3,15);
	He.block<3,3>(0,0) = Eigen::Matrix<float,3,3>::Identity();

	// predicted position
	Eigen::Matrix<float,3,1> p_pred = state(Eigen::seq(0,2));

	// variables to fill
	Eigen::MatrixXf H; // measurement matrix
	Eigen::MatrixXf R; // noise matrix
	Eigen::MatrixXf y_pred; // predicted measurement
	Eigen::MatrixXf y_meas; // actual measurement 

	// measurement and noise matrices depend on encoder only
	H = He;
	R = Re;

	y_pred = Eigen::Matrix<float,3,1>::Zero();
	y_meas = Eigen::Matrix<float,3,1>::Zero();

	// predicted measurement is position 
	y_pred << p_pred(0), p_pred(1), p_pred(2);

	// actual measurement is output position
	y_meas << p_meas(0), p_meas(1), p_meas(2);

	// residual z
	Eigen::Matrix<float,3,1> z = y_meas-y_pred;

	// call filter
	EKF(H,R,z,false);

}

// updates state. general to measurements given appropriately sized 
void EKF(const Eigen::MatrixXf & H, const Eigen::MatrixXf & R, const Eigen::MatrixXf & z, const bool update_bias)
{

	// compute Kalman gain
	Eigen::MatrixXf K = cov*H.transpose()*((R+H*cov*H.transpose()).inverse()); // fix auto later

	// compute state correction
	Eigen::Matrix<float,15,1> dx = K*z;

	// 	rotation update 
	Eigen::Matrix<float,3,1> ro = dx(Eigen::seq(6,8));

	Eigen::Matrix<float,3,3> P(3,3);
	to_skew(ro,P);
	// predicted orientation
	Eigen::Quaternion<float> b_q = Eigen::Quaternion<float>(state[6],state[7],state[8],state[9]);
	Eigen::Matrix<float,3,3> R_nav_to_body = b_q.toRotationMatrix();
	Eigen::Matrix<float,3,3> R_nav_to_body_next = R_nav_to_body*(Eigen::Matrix<float,3,3>::Identity() - P); // (10.67)

	// reorthogonalize matrix (make it into a rotation matrix)... use SVD
	Eigen::JacobiSVD<Eigen::Matrix<float,3,3>> svd(R_nav_to_body_next,Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
	R_nav_to_body_next = svd.matrixU()*((svd.matrixV()).transpose());

	// 	compute next quaternion
	Eigen::Quaternion<float> b_next(R_nav_to_body_next);
	if (abs(ro[0]) > 0.1 || abs(ro[1]) > 0.1 || abs(ro[2]) > 0.1)
	{
		debug("angle update high");
	}

	// update state
	state.block<3,1>(0,0) += dx(Eigen::seq(0,2)); // update position
	state.block<3,1>(3,0) += dx(Eigen::seq(3,5)); // update velocity
	state.block<4,1>(6,0) << b_next.w(), b_next.x(), b_next.y(), b_next.z(); // update quaternion: [w, x, y, z]

	if (true)
	{
		state.block<3,1>(10,0) += dx(Eigen::seq(9,11)); // update accel bias
		state.block<3,1>(13,0) += dx(Eigen::seq(12,14)); // update gyro bias
	}

	// symmetrify
	cov = (cov + cov.transpose())/2.0;
	// update covariance
	cov = (Eigen::Matrix<float,15,15>::Identity()-K*H)*(cov);
	// symmetrify again
	cov = (cov + cov.transpose())/2.0;

	//  update encoder start state: position and orientation TODO atomicity? from @V
	enc_meas << state[0],state[1],state[2], 0, b_next.w(), b_next.x(), b_next.y(), b_next.z();

	if (true)
	{
		Eigen::Quaternion<float> b_next_body_to_nav = b_next.inverse();
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(state(0), state(1), state(2)) );
		//transform.setOrigin( tf::Vector3(0, 0, 0));
		tf::Quaternion q(b_next_body_to_nav.x(),b_next_body_to_nav.y(),b_next_body_to_nav.z(),b_next_body_to_nav.w());
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "IMU"));
	}
}

// encoders callback
void encoders_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{		
	int ticks_l_curr = msg->data[0]; // total ticks left wheel
	int ticks_r_curr = msg->data[1]; // total ticks right wheel
	
	if (first_time_enc)
	{
		ticks_l_prev = ticks_l_curr;
		ticks_r_prev = ticks_r_curr;
		first_time_enc = false;
		return;
	}

	// Compute distance moved by each wheel	
	float Dl = (ticks_l_curr-ticks_l_prev)/filter.ticks_per_m;
	float Dr = (ticks_r_curr-ticks_r_prev)/filter.ticks_per_m;
	float Dc = (Dl+Dr)/2.0;

	// Store previous set of readings
	ticks_l_prev = ticks_l_curr;
	ticks_r_prev = ticks_r_curr;

	// get encoder model position estimate. starts at end of last measurement update
	Eigen::Matrix<float,3,1> pos(enc_meas[0],enc_meas[1],enc_meas[2]);
	// get encoder model orientation
	Eigen::Quaternion<float> b_q = Eigen::Quaternion<float>(enc_meas[4],enc_meas[5],enc_meas[6],enc_meas[7]);

	// b is nav to body
	Eigen::Quaternion<float> b_inv = b_q.inverse();
	Eigen::Matrix<float,3,1> dpos(Dc,0,0);
	pos += b_inv._transformVector(dpos);

	// // store in state 
	enc_meas << pos[0],pos[1],pos[2],0,0,0,0,0;

	// // noise due to distance driven = k*Dl
	Eigen::Matrix<float,2,2> U;
	U << (filter.k*abs(Dl))*(filter.k*abs(Dl)), 0,
	0, (filter.k*abs(Dr))*(filter.k*abs(Dr));

	// pos = pos + R*F*[Dc, 0, 0]'
	// (R*F)*U*(R*F)'
	Eigen::Matrix<float,3,2> F;
	F << 1/2.0,1/2.0, 0,0,0,0;
	Eigen::Matrix<float,3,3> R_body_to_nav = b_inv.toRotationMatrix();

	// transform noise covariance
	Re = (R_body_to_nav*F)*U*(R_body_to_nav*F).transpose();

	if (false)
	{
		Eigen::Quaternion<float> b_next_body_to_nav = b_inv;

		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(pos[0], pos[1], pos[2]) );
		tf::Quaternion q(b_next_body_to_nav.x(),b_next_body_to_nav.y(),b_next_body_to_nav.z(),b_next_body_to_nav.w());
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "IMU"));
	}

	encoderMeasurementUpdate();

}

// imu callback
// updates orientation and position using quaternion math and physics kinematic equations
// Prediction Step/Time Propogation/State Update/Vehicle Kinematics step
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	// Uncomment to time things
	//filter.timer.PrintDt("Ignore"); 
	//filter.timer.PrintDt("IMU"); 

	// IMU data
	geometry_msgs::Vector3 w = msg->angular_velocity;
	geometry_msgs::Vector3 f = msg->linear_acceleration;
	Eigen::Matrix<float,3,1> w_nb(w.x,w.y,w.z);
	Eigen::Matrix<float,3,1> f_b(f.x,f.y,f.z);

	/* Update State */

	// current position
	Eigen::Matrix<float,3,1> p = state(Eigen::seq(0,2));

	// current velocity
	Eigen::Matrix<float,3,1> v = state(Eigen::seq(3,5));

	// current orientation
	Eigen::Matrix<float,4,1> b = state(Eigen::seq(6,9));
	Eigen::Quaternion<float> b_prev = Eigen::Quaternion<float>(b(0),b(1),b(2),b(3)); // w, x, y, z //could also be called b_current

	// current accel bias
	Eigen::Matrix<float,3,1> x_a = state(Eigen::seq(10,12));

	// current gyro bias
	Eigen::Matrix<float,3,1> x_g = state(Eigen::seq(13,15));

	// subtract out gyroscope bias. also w_bn = (-w_nb)
	Eigen::Matrix<float,3,1> w_bn = -1*(w_nb-x_g);
	float w_norm = w_bn.norm();

	// subtract out accelerometer bias //TODO can be more complicated if needed
	f_b = f_b - x_a;

	// differential rotation: [w, x, y, z]
	Eigen::Matrix<float,3,1> xyz = sin(w_norm*filter.dt/2.0)*w_bn/w_norm;
	Eigen::Quaternion<float> db = Eigen::Quaternion<float>(cos(w_norm*filter.dt/2.0), xyz[0], xyz[1], xyz[2]);

	// update orientation
	Eigen::Quaternion<float> b_next = db*b_prev;

	// get average quaternion by interpolation //want measurement btw. now and next timestep
	Eigen::Quaternion<float> b_avg = b_prev.slerp(0.5,b_next);

	// b is the nav to body transformation. we need body to nav transformation -> invert 
	Eigen::Quaternion<float> b_body_to_nav_avg = b_avg.inverse(); // for specific force (5.9 Principles of GNSS book)

	// rotate specific force into inertial frame
	Eigen::Matrix<float,3,1> f_i = b_body_to_nav_avg._transformVector(f_b);

	// gravity vector //TODO make static to be constant
	Eigen::Matrix<float,3,1> g_vec(0,0,filter.g);

	// get acceleration in inertial frame. (acceleration of body wrt inertial frame in inertial frame)
	// inertial frame is already affected by gravity. 
	// Critical Point of algo. Will be very bad if your orientation is wrong
	Eigen::Matrix<float,3,1> a_i = f_i - g_vec;

	// The next two lines of code are PHYSICS EQUATIONS
	
	// update position (5.16 Principles of GNSS book)
	p = p + v*filter.dt + 0.5*a_i*filter.dt*filter.dt;

	// update velocity
	v = v + a_i*filter.dt;


	// store in state -> this is time propagation step. 
	state << p(0),p(1),p(2), v(0),v(1),v(2), b_next.w(),b_next.x(),b_next.y(),b_next.z(), x_a(0),x_a(1),x_a(2), x_g(0),x_g(1),x_g(2);

	/* Update covariance */
	Eigen::Matrix<float,3,3> R_body_to_nav_next = b_next.inverse().toRotationMatrix();

	// compute state transition matrix Phi
	Eigen::Matrix<float,15,15> Phi(15,15);
	computePhi(f_i, R_body_to_nav_next, Phi); // TODO Review
	//TODO do the Van Load computePhi (@A)

	// compute Qdk (discrete noise). Qdk is 15 x 15
	Eigen::Matrix<float,15,15> Qdk(15,15);
	computeQdk(R_body_to_nav_next, Qdk); // TODO Review

	// update covariance (15x15)
	cov = Phi*cov*Phi.transpose()+Qdk;

	/* Measurement update using accelerometer to correct roll and pitch */
	// if 50 accelerometer readings were close enough to the gravity vector, robot is stationary
	//  check if current measurement is stationary
	if (abs(f_b.norm() - filter.g) < 0.03) // tuned. test: you should never be able to hold the imu in your hand and have an update. //TODO tune 0.03 @V
	{
		accel_counter++;
		g_pred_sum += R_body_to_nav_next*(f_b); // R*(x_a-y_a) TODO: CHECK THIS

	} else {
		accel_counter = 0;
		g_pred_sum = Eigen::Matrix<float,3,1>::Zero();
		rover_stationary = false;
	}
	// if 50 consecutive stationary, use accel_data //TODO tune @V. It currently is at about 0.25 seconds
	if (accel_counter == 50)
	{
		// predict gravity in navigation frame and store prediction in global variable.
		g_pred = g_pred_sum/50.0; // averaging
		accel_counter = 0;
		g_pred_sum = Eigen::Matrix<float,3,1>::Zero();
		stationaryMeasurementUpdate(R_body_to_nav_next);

	}

	if (false)
	{
		Eigen::Quaternion<float> b_next_body_to_nav = b_next.inverse();
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(state(0), state(1), state(2)) );
		tf::Quaternion q(b_next_body_to_nav.x(),b_next_body_to_nav.y(),b_next_body_to_nav.z(),b_next_body_to_nav.w());
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "IMU"));
	}

}
//TODO include IMU's orientation into this EKF (as if it was a sun sensor. @V

void initialize_ekf(ros::NodeHandle &n)
{
	ROS_INFO("ekf: waiting for initialization service");

	// create client for service
	ros::ServiceClient client = n.serviceClient<encoder_imu_ekf_ros::initRequest>("initialize_ekf");

	// instantiate service class
	encoder_imu_ekf_ros::initRequest srv; //gives initial orientation (when robot stationary) 

	// call the service
	if (!client.waitForExistence(ros::Duration(-1)))
	{
		ROS_ERROR("initialize_ekf didn't send data");
	}
	if (client.call(srv)) //see handle_init_ekf
	{
		ROS_INFO("initialize_ekf responded with data."); 
		// store received data
		geometry_msgs::Quaternion b = srv.response.init_orientation;
		Eigen::Vector3d x_g = {srv.response.gyro_bias[0].data, srv.response.gyro_bias[1].data, srv.response.gyro_bias[2].data}; //biases
		//TODO accelearation biases

		// filter rate parameters
		int num_data, hz; // number of data points used to initialize, imu hz
		n.param("num_data",num_data,1000); //num init pts
		n.param("imu_hz",hz,200);
		filter.dt = 1.0/hz;
		filter.num_data = num_data;
		int T = num_data/hz;  //number of measurements over rate of IMU

		// initialize noise terms //can put in launch file
		float sigma_xg, sigma_nug, sigma_xa, sigma_nua;
		n.param<float>("sigma_xg",sigma_xg,0.00000290); // Gyro (rate) random walk
		n.param<float>("sigma_nug",sigma_nug,0.00068585); // rad/s/rt_Hz, Gyro white noise
		n.param<float>("sigma_xa",sigma_xa,0.00001483);  // Accel (rate) random walk m/s3 1/sqrt(Hz)
		n.param<float>("sigma_nua",sigma_nua,0.00220313); // accel white noise

		// noise matrix for IMU (Q) //TODO can optimize
		for (int i = 0; i < 3; i++)
		{
			filter.Q(i,i) = sigma_nua*sigma_nua;
			filter.Q(3+i,3+i) = sigma_nug*sigma_nug;
			filter.Q(6+i,6+i) = sigma_xa*sigma_xa;
			filter.Q(9+i,9+i) = sigma_xg*sigma_xg;
		}

		// noise matrix for accelerometer (Ra)
		filter.Ra = Eigen::Matrix<float,3,3>::Identity(3,3)*(sigma_nua*sigma_nua);

		// gravity vector
		n.param<float>("g",filter.g,9.8021);

		// encoder slip model, % slip
		n.param<float>("k",filter.k,0.05); //TODO  keep editing and discover better definition of this 
					//note that the slip can only cause you to overshoot your estimate

		// robot dimensional parameters
		n.param<float>("L",filter.L, 0.6096); // base width (m)
		n.param<float>("R",filter.R, 0.127); // wheel radius (m)
		n.param<int>("ticks_per_rev", filter.ticks_per_rev, 1440); // wheel encoder parameters
		n.param<float>("ticks_per_m", filter.ticks_per_m,filter.ticks_per_rev/(PI*2*filter.R));

		//http://web.mit.edu/2.05/www/Handout/HO2.PDF, euler angles != RPY. xyz euler = zyx rpy
		// Eigen::Quaternion<float> b_q = Eigen::Quaternion<float>(b.w,b.x,b.y,b.z);
		// Eigen::Matrix<float,3,1> euler_angles = b_q.inverse().toRotationMatrix().eulerAngles(0,1,2); // 'xyz'
		// float yaw_init = euler_angles[2];

		// initialize state: [p, v, b, x_a, x_g] = [position, velocity, quaternion, accel bias, gyro bias],  size 16
		state << 0,0,0,0,0,0,b.w,b.x,b.y,b.z,0,0,0,x_g[0],x_g[1],x_g[2];

		// initialize covariance
		cov.block<2,2>(6,6) = (sigma_nua/filter.g)*(sigma_nua/filter.g)/T*Eigen::Matrix<float,2,2>::Identity(); //orientation uncertainty
		cov.block<3,3>(12,12) = (sigma_nug)*(sigma_nug)/T*Eigen::Matrix<float,3,3>::Identity(); //gyro bias uncertainty
		//TODO accelerometer initial uncertainty (after initializing accelerometer bias)

		// initialize encoder model state [p,0,b] = [position, 0, orientation] 
		 		//0 was the predicted yaw from diff_drive model
		// encoder acts like a sensor initialized at end of every measurement update 
				//orientation is an add-on (TODO Check: not nesec from the encoders)
		enc_meas << 0,0,0,0,b.w,b.x,b.y,b.z;

	}
	else
	{
		ROS_ERROR("Failed to call service initialize_ekf.");
	}
}


int main(int argc, char **argv)
{
	ROS_INFO("EKF node started.");

	ros::init(argc, argv, "ekf");

	ros::NodeHandle n;

	// initialize ekf
	initialize_ekf(n);

	// encoder callback
	ros::Subscriber sub_encoders = n.subscribe("wheels", 0, encoders_callback);

	// imu callback
	ros::Subscriber sub_imu = n.subscribe("/imu/data", 0, imu_callback);

	ros::spin();

	return 0;
}
