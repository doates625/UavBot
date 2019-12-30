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
using CppUtil::square;

/**
 * Namespace Definitions
 */
namespace Controller
{
	// Constants
	const float f_ctrl = 50.0f;
	const float t_ctrl_us = 1e6f / f_ctrl;
	const float mass = 0.552f;
	const float gravity = 9.807f;
	const float q_pole = -30.0f;
	const float th_min = 0.1f;
	const float th_max = 0.9f;

	// Derived constants
	const float a_min = (4.0f * force_max / mass) * th_min;
	const float a_max = (4.0f * force_max / mass) * th_max;
	const float k_q = square(q_pole);
	const float k_w = -2.0f * q_pole;

	// Vectors and matrices
	Vector<3> x_hat, y_hat, z_hat;
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
		// Initialize arrays
		x_hat(0) = 1.0f;
		y_hat(1) = 1.0f;
		z_hat(2) = 1.0f;
		M_alp(0, 0) = +3.0913978e-03;
		M_alp(1, 0) = -3.0913978e-03;
		M_alp(2, 0) = +3.0913978e-03;
		M_alp(3, 0) = -3.0913978e-03;
		M_alp(0, 1) = -3.5483871e-03;
		M_alp(1, 1) = -3.5483871e-03;
		M_alp(2, 1) = +3.5483871e-03;
		M_alp(3, 1) = +3.5483871e-03;
		M_alp(0, 2) = +1.0181818e-02;
		M_alp(1, 2) = -1.0181818e-02;
		M_alp(2, 2) = -1.0181818e-02;
		M_alp(3, 2) = +1.0181818e-02;
		M_acc(0, 0) = +1.3800000e-01;
		M_acc(1, 0) = +1.3800000e-01;
		M_acc(2, 0) = +1.3800000e-01;
		M_acc(3, 0) = +1.3800000e-01;

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
	// Get state data from imu
	Quat q_act = Imu::get_quat(); // HAVE NOT VALIDATED
	Vector<3> omega = Imu::get_omega(); // HAVE NOT VALIDATED
	
	// Get commands from Bluetooth
	Vector<3> acc_cmd = Bluetooth::get_acc_cmd(); // HAVE NOT VALIDATED
	float tz_cmd = Bluetooth::get_heading_cmd(); // HAVE NOT VALIDATED

	// Adjust for gravity
	acc_cmd(2) += gravity;
	
	// Acceleration limiting
	acc_cmd(2) = clamp(acc_cmd(2), a_min, a_max);
	float norm_xy = sqrtf(square(acc_cmd(0)) + square(acc_cmd(1))); // LinearCpp: Add sub-indexing
	float norm_xy_max = sqrtf(square(a_max) - square(acc_cmd(2))); // Make 'sq' (function 'square' is too clunky)
	float p = norm_xy_max / norm_xy;
	if (p < 1.0f)
	{
		acc_cmd(0) *= p; // LinearCpp: Add sub-indexing and *= operator
		acc_cmd(1) *= p;
	}

	// Orientation calculation
	Quat q_cmd;
	Quat qz(z_hat, tz_cmd); // FOUND SOMETHING 
	float norm_acc = norm(acc_cmd);
	if (norm_acc > 0)
	{
		Vector<3> acc_hat = (1.0f / norm_acc) * acc_cmd; // LinearCpp: add vector-float division
		float cz = cos(tz_cmd);
		float sz = sin(tz_cmd);
		float tx = asinf(sz*acc_hat(0) - cz*acc_hat(1));
		float ty = asinf((cz*acc_hat(0) + sz*acc_hat(1))/cosf(tx));
		Quat qy(y_hat, ty);
		Quat qx(x_hat, tx);
		q_cmd = qz * qy * qx;
	}
	else
	{
		q_cmd = qz;
	}

	// Acceleration magnitude cmd
	Vector<3> n_hat = q_act * z_hat;
	float acc_mag = acc_cmd(2) / n_hat(2);
	acc_mag = clamp(acc_mag, a_min, a_max);

	// Quaternion orientation control
	Quat q_err = inv(q_cmd) * q_act;
	if (q_err.w < 0.0f)
	{
		q_err.w = -q_err.w;	// LinearCpp - Make op= for MatrixExp<4, 1>
		q_err.x = -q_err.x;
		q_err.y = -q_err.y;
		q_err.z = -q_err.z;
	}
	Vector<3> q_err_vec;
	q_err_vec(0) = q_err.x;
	q_err_vec(1) = q_err.y;
	q_err_vec(2) = q_err.z;
	Vector<3> alp_cmd = -(k_q * q_err_vec + k_w * omega);

	// Force regulator controller
	Matrix<1, 1> acc_mag_mat;
	acc_mag_mat(0, 0) = acc_mag;
	Vector<4> f_alp = M_alp * alp_cmd;
	Vector<4> f_acc = M_acc * acc_mag_mat;
	float p_min = 1.0f;
	for (uint8_t i = 0; i < 4; i++)
	{
		float p = 1.0f;
		if (f_alp(i) > 0.0f)
		{
			p = (force_max - f_acc(i)) / f_alp(i);
		}
		else if (f_alp(i) < 0.0f)
		{
			p = (force_min - f_acc(i)) / f_alp(i);
		}
		
		if ((0.0f < p) && (p < p_min))
		{
			p_min = p;
		}
	}
	Vector<4> forces = p_min * f_alp + f_acc;

	// Clamp limit (TODO - try removing)
	for (uint8_t i = 0; i < 4; i++)
	{
		forces(i) = clamp(forces(i), force_min, force_max);
	}

	// Set motor forces
	Motors::set_forces(forces);
}
