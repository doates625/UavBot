/**
 * @file Controller.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Controller.h"
#include <Imu.h>
#include <Bluetooth.h>
#include <Props.h>
#include <CppUtil.h>
#include <PID.h>
using CppUtil::clamp;
using CppUtil::sqa;

/**
 * Namespace Definitions
 */
namespace Controller
{
	// Control Constants
	const float f_ctrl = 50.0f;		// Control freq [Hz]
	const float thr_min = 0.20f;	// Min linear throttle
	const float thr_max = 0.60f;	// Max linear throttle

	// Quat X-axis Gains
	const float qx_kp = +0.188f;	// P-gain
	const float qx_ki = +0.314f;	// I-gain
	const float qx_kd = +0.038f;	// D-gain
	const float qx_ff = +0.000f;	// Feed-forward

	// Quat Y-axis Gains
	const float qy_kp = +0.216f;	// P-gain
	const float qy_ki = +0.361f;	// I-gain
	const float qy_kd = +0.043f;	// D-gain
	const float qy_ff = +0.000f;	// Feed-forward

	// Quat Z-axis Gains
	const float qz_kp = +0.621f;	// P-gain
	const float qz_ki = +1.035f;	// I-gain
	const float qz_kd = +0.124f;	// D-gain
	const float qz_ff = +0.000f;	// Feed-forward

	// Derived Constants
	const float t_ctrl_us = 1e6f / f_ctrl;

	// Quaternion PID Controllers
	PID quat_x_pid(qx_kp, qx_ki, qx_kd, -HUGE_VALF, +HUGE_VALF, f_ctrl);
	PID quat_y_pid(qy_kp, qy_ki, qy_kd, -HUGE_VALF, +HUGE_VALF, f_ctrl);
	PID quat_z_pid(qz_kp, qz_ki, qz_kd, -HUGE_VALF, +HUGE_VALF, f_ctrl);
	bool quat_sat = false;

	// Throtttle vectors and matrices
	Matrix<4, 3> N_ang;
	Matrix<4, 1> N_lin;
	Vector<4> thr_props;

	// Subsystem init flag
	bool init_complete = false;
}

/**
 * @brief Inits controller
 */
void Controller::init()
{
	if (!init_complete)
	{
		// Initialize angular prop matrix
		N_ang(0, 0) = +1.000000e+00f;
		N_ang(0, 1) = -1.000000e+00f;
		N_ang(0, 2) = +1.000000e+00f;
		N_ang(1, 0) = -1.000000e+00f;
		N_ang(1, 1) = -1.000000e+00f;
		N_ang(1, 2) = -1.000000e+00f;
		N_ang(2, 0) = +1.000000e+00f;
		N_ang(2, 1) = +1.000000e+00f;
		N_ang(2, 2) = -1.000000e+00f;
		N_ang(3, 0) = -1.000000e+00f;
		N_ang(3, 1) = +1.000000e+00f;
		N_ang(3, 2) = +1.000000e+00f;

		// Initialize linear prop matrix
		N_lin(0, 0) = +1.000000e+00f;
		N_lin(1, 0) = +1.000000e+00f;
		N_lin(2, 0) = +1.000000e+00f;
		N_lin(3, 0) = +1.000000e+00f;

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Runs one control loop iteration to calculate prop forces
 */
void Controller::update()
{
	// Get state from IMU
	Quat ang_pos = Imu::get_ang_pos();

	// Get commands from Bluetooth
	Quat ang_pos_cmd = Bluetooth::get_ang_pos_cmd();
	float thr_lin_cmd = Bluetooth::get_thr_lin_cmd();
	Vector<1> thr_lin_d(clamp(thr_lin_cmd, thr_min, thr_max));
	Vector<4> thr_lin = N_lin * thr_lin_d;

	// Quaternion control
	Quat ang_err = inv(ang_pos_cmd) * ang_pos;
	if (ang_err.w < 0.0f) ang_err = -ang_err;
	Vector<3> thr_ang_d;
	thr_ang_d(0) = quat_x_pid.update(-ang_err.x, qx_ff, quat_sat);
	thr_ang_d(1) = quat_y_pid.update(-ang_err.y, qy_ff, quat_sat);
	thr_ang_d(2) = quat_z_pid.update(-ang_err.z, qz_ff, quat_sat);
	Vector<4> thr_ang = N_ang * thr_ang_d;

	// Throttle anti-windup
	float p_min = 1.0f;
	quat_sat = false;
	for (uint8_t i = 0; i < 4; i++)
	{
		float p =
			(thr_ang(i) > 0.0f) ? ((1.0f - thr_lin(i)) / thr_ang(i)) :
			(thr_ang(i) < 0.0f) ? ((0.0f - thr_lin(i)) / thr_ang(i)) :
			1.0f;
		if ((0.0f < p) && (p < p_min))
		{
			p_min = p;
			quat_sat = true;
		}
	}
	thr_props = p_min * thr_ang + thr_lin;
}

/**
 * @brief Resets controller to startup state
 */
void Controller::reset()
{
	quat_x_pid.reset();
	quat_y_pid.reset();
	quat_z_pid.reset();
	quat_sat = false;
	thr_props = Vector<4>();
}

/**
 * @brief Returns prop throttles [0, 1]
 */
const Vector<4>& Controller::get_thr_props()
{
	return thr_props;
}
