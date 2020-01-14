/**
 * @file Controller.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Controller.h"
#include <Imu.h>
#include <Bluetooth.h>
#include <Props.h>
#include <Params.h>
#include <CppUtil.h>
#include <PID.h>
using CppUtil::clamp;
using CppUtil::sqa;

/**
 * Namespace Definitions
 */
namespace Controller
{
	// Timing constants
	const float f_ctrl_hz = 50.0f;
	const float t_ctrl_us = 1e6f / f_ctrl_hz;

	// Flight parameters
	float thr_min = 0.0f;	// Min linear throttle [0, 1]
	float thr_max = 0.0f;	// Max linear throttle [0, 1]
	float qx_ff = 0.0f;		// Quat x-axis feed-forward [thr]
	float qy_ff = 0.0f;		// Quat y-axis feed-forward [thr]
	float qz_ff = 0.0f;		// Quat z-axis feed-forward [thr]

	// Quaternion PID Controllers
	PID qx_pid(0.0f, 0.0f, 0.0f, -HUGE_VALF, +HUGE_VALF, f_ctrl_hz);
	PID qy_pid(0.0f, 0.0f, 0.0f, -HUGE_VALF, +HUGE_VALF, f_ctrl_hz);
	PID qz_pid(0.0f, 0.0f, 0.0f, -HUGE_VALF, +HUGE_VALF, f_ctrl_hz);
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
 * @brief Sets flight parameters
 */
void Controller::set_params(const Params& params)
{
	// Update throttle limits
	thr_min = params.thr_min;
	thr_max = params.thr_max;

	// Update Quat-x PID gains
	qx_pid.set_k_p(params.qx_kp);
	qx_pid.set_k_i(params.qx_ki);
	qx_pid.set_k_d(params.qx_kd);
	qx_ff = params.qx_ff;

	// Update Quat-y PID gains
	qy_pid.set_k_p(params.qy_kp);
	qy_pid.set_k_i(params.qy_ki);
	qy_pid.set_k_d(params.qy_kd);
	qy_ff = params.qy_ff;

	// Update Quat-z PID gains
	qz_pid.set_k_p(params.qz_kp);
	qz_pid.set_k_i(params.qz_ki);
	qz_pid.set_k_d(params.qz_kd);
	qz_ff = params.qz_ff;
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
	thr_ang_d(0) = qx_pid.update(-ang_err.x, qx_ff, quat_sat);
	thr_ang_d(1) = qy_pid.update(-ang_err.y, qy_ff, quat_sat);
	thr_ang_d(2) = qz_pid.update(-ang_err.z, qz_ff, quat_sat);
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
	qx_pid.reset();
	qy_pid.reset();
	qz_pid.reset();
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
