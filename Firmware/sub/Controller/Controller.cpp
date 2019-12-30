/**
 * @file Controller.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Controller.h"
#include <Imu.h>
#include <Bluetooth.h>
#include <Motors.h>
#include <CppUtil.h>
using Motors::force_min;
using Motors::force_max;
using CppUtil::clamp;
using CppUtil::sqa;

/**
 * Namespace Definitions
 */
namespace Controller
{
	// Constants
	const float f_ctrl = 50.0f;		// Control freq [Hz]
	const float mass = 0.552f;		// UAV mass [kg]
	const float gravity = 9.807f;	// Gravity [m/s^2]
	const float q_pole = -30.0f;	// Quat pole [s^-1]
	const float th_min = 0.1f;		// Min thrust [N/N]
	const float th_max = 0.9f;		// Max thrust [N/N]

	// Derived constants
	const float t_ctrl_us = 1e6f / f_ctrl;
	const float acc_max = (4.0f * force_max / mass);
	const float acc_mag_min = acc_max * th_min;
	const float acc_mag_max = acc_max * th_max;
	const float acc_mag_max_sq = sqa(acc_mag_max);
	const float alp_gain_q = sqa(q_pole);
	const float alp_gain_w = -2.0f * q_pole;

	// Vectors and matrices
	Vector<3> x_hat;
	Vector<3> y_hat;
	Vector<3> z_hat;
	Matrix<4, 3> M_alp;
	Matrix<4, 1> M_acc;

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Inits controller
 */
void Controller::init()
{
	if (!init_complete)
	{
		// Initialize x unit vector
		x_hat(0) = +1.000000e+00f;
		x_hat(1) = +0.000000e+00f;
		x_hat(2) = +0.000000e+00f;

		// Initialize y unit vector
		y_hat(0) = +0.000000e+00f;
		y_hat(1) = +1.000000e+00f;
		y_hat(2) = +0.000000e+00f;

		// Initialize z unit vector
		z_hat(0) = +0.000000e+00f;
		z_hat(1) = +0.000000e+00f;
		z_hat(2) = +1.000000e+00f;

		// Initialize angular mass matrix
		M_alp(0, 0) = +3.091398e-03f;
		M_alp(0, 1) = -3.548387e-03f;
		M_alp(0, 2) = +1.018182e-02f;
		M_alp(1, 0) = -3.091398e-03f;
		M_alp(1, 1) = -3.548387e-03f;
		M_alp(1, 2) = -1.018182e-02f;
		M_alp(2, 0) = +3.091398e-03f;
		M_alp(2, 1) = +3.548387e-03f;
		M_alp(2, 2) = -1.018182e-02f;
		M_alp(3, 0) = -3.091398e-03f;
		M_alp(3, 1) = +3.548387e-03f;
		M_alp(3, 2) = +1.018182e-02f;

		// Initialize linear mass matrix
		M_acc(0, 0) = +1.380000e-01f;
		M_acc(1, 0) = +1.380000e-01f;
		M_acc(2, 0) = +1.380000e-01f;
		M_acc(3, 0) = +1.380000e-01f;

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Runs one control loop iteration
 * 
 * Reference: Matlab\UavMatlabSim.m
 */
void Controller::update()
{
	// Get state and commands
	Quat q_act = Imu::get_quat();
	Vector<3> omega = Imu::get_omega();
	Vector<3> acc_cmd = Bluetooth::get_acc_cmd();
	float tz_cmd = Bluetooth::get_heading_cmd();

	// Adjust accel for gravity
	acc_cmd(2) += gravity;
	
	// Accel command limiting
	acc_cmd(2) = clamp(acc_cmd(2), acc_mag_min, acc_mag_max);
	float norm_xy = hypot(acc_cmd(0), acc_cmd(1));
	float norm_xy_max = sqrtf(acc_mag_max_sq - sqa(acc_cmd(2)));
	float p = norm_xy_max / norm_xy;
	if (p < 1.0f)
	{
		acc_cmd(0) *= p; // LinearCpp: Add sub-indexing and *= operator
		acc_cmd(1) *= p;
	}

	// Orientation calculation
	Quat q_z(z_hat, tz_cmd);
	Quat q_cmd = q_z;
	float norm_acc = norm(acc_cmd);
	if (norm_acc > 0)
	{
		Vector<3> acc_hat = acc_cmd / norm_acc;
		float cos_z = cos(tz_cmd);
		float sin_z = sin(tz_cmd);
		float t_x = asinf(sin_z*acc_hat(0) - cos_z*acc_hat(1));
		float t_y = asinf((cos_z*acc_hat(0) + sin_z*acc_hat(1)) / cosf(t_x));
		Quat q_y(y_hat, t_y);
		Quat q_x(x_hat, t_x);
		q_cmd = q_z * q_y * q_x;
	}

	// Acceleration magnitude cmd
	Vector<3> n_hat = q_act * z_hat;
	float acc_mag_cmd = acc_cmd(2) / n_hat(2);
	acc_mag_cmd = clamp(acc_mag_cmd, acc_mag_min, acc_mag_max);

	// Quaternion control
	Quat q_err = inv(q_cmd) * q_act;
	if (q_err.w < 0.0f) q_err = -q_err;
	Vector<3> q_err_vec;
	q_err_vec(0) = q_err.x;	// LinearCpp: Sub-indexing will clean this up
	q_err_vec(1) = q_err.y;
	q_err_vec(2) = q_err.z;
	Vector<3> alp_cmd = -(alp_gain_q * q_err_vec + alp_gain_w * omega);

	// Force regulator controller
	Matrix<1, 1> acc_mag_mat;	// LinearCpp: Add cast float to Matrix<1, 1> and Vector<1, 1>
	acc_mag_mat(0, 0) = acc_mag_cmd;
	Vector<4> f_alp = M_alp * alp_cmd;
	Vector<4> f_acc = M_acc * acc_mag_mat;
	float p_min = 1.0f;
	for (uint8_t i = 0; i < 4; i++)
	{
		float p =
			(f_alp(i) > 0.0f) ? ((force_max - f_acc(i)) / f_alp(i)) :
			(f_alp(i) < 0.0f) ? ((force_min - f_acc(i)) / f_alp(i)) :
			1.0f;
		if ((0.0f < p) && (p < p_min)) p_min = p;	
	}
	Vector<4> forces = p_min * f_alp + f_acc;

	// Set motor forces
	Motors::set_forces(forces);
}
