#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <chrono>
#include <string.h>
#include <iostream>

// timer class to time parts of
class Timer 
{
public:
	Timer()
	{
		last = std::chrono::steady_clock::now();
	}
	float PrintDt(std::string label)
	{
		std::cout << label << ": " << CalculateTimeDiff() << std::endl;
	}

private:
	std::chrono::steady_clock::time_point last;
	float CalculateTimeDiff()
	{
		const auto old = last;
		last = std::chrono::steady_clock::now();
		const  std::chrono::duration<float> frame_time = last-old;
		return frame_time.count();
	}
};

 // struct which contains constant parameters used in algorithm
struct EKF_struct {
		// global variables in filter
		Eigen::Matrix<float,12,12> Q = Eigen::Matrix<float,12,12>::Zero(); // noise matrix for IMU
		Eigen::Matrix<float,3,3> Ra = Eigen::Matrix<float,3,3>::Zero(); // noise matrix for accelerometer
		float g; // gravity
		float k; // slip constant
		float dt; // time step
		int num_data; // number of data collected for initialization
		float L; // wheelbase width (m)
		float R; // wheel radius (m)
		int ticks_per_rev;
		float ticks_per_m;
		Timer timer;
};


const double PI = 2*acos(0.0);

// variables used throughout algorithm 
Eigen::Matrix<float,16, 1> state(16,1); // state
Eigen::Matrix<float, 15, 15> cov = Eigen::Matrix<float,15,15>::Zero(); // covariance
Eigen::Matrix<float, 8, 1> enc_meas(8,1); // encoder "measurement"
EKF_struct filter; // filter object

// variables used for measurement update if rover stationary
bool rover_stationary = false; // affects the measurement update; is rover stationary
int accel_counter = 0; //num acc meas when stat
Eigen::Matrix<float,3,1> g_pred(3,1); //predicted g
Eigen::Matrix<float,3,1> g_pred_sum(3,1); //predicted g over time

// variables used for measurement update for encoders
bool first_time_enc = true;
int ticks_l_prev;
int ticks_r_prev;
Eigen::Matrix<float,3,3> Re = Eigen::Matrix<float,3,3>::Zero(); // measurement noise matrix

// printing function
void debug(const std::string & str);

// convert vector to skew symmetric matrix
void to_skew(const Eigen::Matrix<float,3,1>& v, Eigen::Matrix<float,3,3> &m);

// state transition matrix
void computePhi(const Eigen::Matrix<float,3,1> &f_i, const Eigen::Matrix<float,3,3> &R_body_to_nav_next, Eigen::Matrix<float,15,15> &Phi);

// encoder model noise
void computeQdk(const Eigen::Matrix<float,3,3> &R_body_to_nav_next, Eigen::Matrix<float,15,15> &Qdk);

// measurement update using rover kinematic model
void encoderMeasurementUpdate();

// measurement update using gravity prediction
void stationaryMeasurementUpdate(const Eigen::Matrix<float,3,3> & R_body_to_nav);

// general Kalman filter
void EKF(const Eigen::MatrixXf & H, const Eigen::MatrixXf & R, const Eigen::MatrixXf & z, const bool update_bias);
//void EKF(const Eigen::MatrixXf & H, const Eigen::MatrixXf & R, const Eigen::MatrixXf & z, const bool update_bias, const bool update_pos, const bool update_orientation);
