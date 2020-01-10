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
using Props::f_prop_min;
using Props::f_prop_max;
using CppUtil::clamp;
using CppUtil::sqa;

/**
 * Namespace Definitions
 */
namespace Controller
{
	// Physical Constants
	const float I_xx = 1.15e-03f;	// Inertia x-axis [kg*m^2]
	const float I_yy = 1.32e-03f;	// Inertia y-axis [kg*m^2]
	const float I_zz = 2.24e-03f;	// Inertia z-axis [kg*m^2]
	const float mass = 0.546f;		// Total mass [kg]
	const float gravity = 9.807f;	// Gravity [m/s^2]

	// Control Constants
	const float f_ctrl = 50.0f;		// Control freq [Hz]
	const float s_qx = -5.0f;		// Quat x-axis pole [s^-1]
	const float s_qy = -5.0f;		// Quat y-axis pole [s^-1]
	const float s_qz = -3.0f;		// Quat z-axis pole [s^-1]
	const float s_az = -6.0f;		// Accel z-axis pole [s^-1]
	const float fr_min = 0.1f;		// Min prop thrust ratio [N/N]
	const float fr_max = 0.9f;		// Max prop thrust ratio [N/N]

	// Derived constants
	const float t_ctrl_s = 1.0f / f_ctrl;
	const float t_ctrl_us = 1e6f * t_ctrl_s;
	const float acc_max = (4.0f * f_prop_max / mass);
	const float acc_mag_min = acc_max * fr_min;
	const float acc_mag_max = acc_max * fr_max;
	const float acc_mag_max_sq = sqa(acc_mag_max);
	const float f_lin_min = f_prop_max * fr_min;
	const float f_lin_max = f_prop_max * fr_max;

	// Quat x-axis PID controller
	const float qx_kp = +6.0f * I_xx * powf(s_qx, 2.0f);
	const float qx_ki = -2.0f * I_xx * powf(s_qx, 3.0f);
	const float qx_kd = -6.0f * I_xx * powf(s_qx, 1.0f);
	PID quat_x_pid(qx_kp, qx_ki, qx_kd, -HUGE_VALF, +HUGE_VALF, f_ctrl);

	// Quat y-axis PID controller
	const float qy_kp = +6.0f * I_yy * powf(s_qy, 2.0f);
	const float qy_ki = -2.0f * I_yy * powf(s_qy, 3.0f);
	const float qy_kd = -6.0f * I_yy * powf(s_qy, 1.0f);
	PID quat_y_pid(qy_kp, qy_ki, qy_kd, -HUGE_VALF, +HUGE_VALF, f_ctrl);

	// Quat z-axis PID controller
	const float qz_kp = +6.0f * I_zz * powf(s_qz, 2.0f);
	const float qz_ki = -2.0f * I_zz * powf(s_qz, 3.0f);
	const float qz_kd = -6.0f * I_zz * powf(s_qz, 1.0f);
	PID quat_z_pid(qz_kp, qz_ki, qz_kd, -HUGE_VALF, +HUGE_VALF, f_ctrl);

	// Accel z-axis PID controller
	const float az_kp = 0.0f;
	const float az_ki = -0.25f * mass * s_az;
	const float az_kd = 0.0f;
	PID acc_z_pid(az_kp, az_ki, az_kd, f_lin_min, f_lin_max, f_ctrl);

	// Vectors and matrices
	Vector<3> x_hat;
	Vector<3> y_hat;
	Vector<3> z_hat;
	Matrix<4, 3> D_bar;
	Vector<4> f_props;

	// Quaternion PID saturation flag
	bool quat_sat = false;

	// Flags
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

		// Initialize inverse moment arm matrix
		D_bar(0, 0) = +2.688172e+00f;
		D_bar(0, 1) = -2.688172e+00f;
		D_bar(0, 2) = +4.545455e+00f;
		D_bar(1, 0) = -2.688172e+00f;
		D_bar(1, 1) = -2.688172e+00f;
		D_bar(1, 2) = -4.545455e+00f;
		D_bar(2, 0) = +2.688172e+00f;
		D_bar(2, 1) = +2.688172e+00f;
		D_bar(2, 2) = -4.545455e+00f;
		D_bar(3, 0) = -2.688172e+00f;
		D_bar(3, 1) = +2.688172e+00f;
		D_bar(3, 2) = +4.545455e+00f;

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Runs one control loop iteration to calculate prop forces
 */
void Controller::update()
{
	// Get state and commands
	Quat ang_pos = Imu::get_ang_pos();
	Vector<3> lin_acc = Imu::get_lin_acc();
	Vector<3> lin_acc_cmd = Bluetooth::get_lin_acc_cmd();
	float ang_z_cmd = Bluetooth::get_ang_z_cmd();

	// Adjust accel for gravity
	lin_acc_cmd(2) += gravity;
	
	// Accel command limiting
	lin_acc_cmd(2) = clamp(lin_acc_cmd(2), acc_mag_min, acc_mag_max);
	float norm_xy = hypot(lin_acc_cmd(0), lin_acc_cmd(1));
	float norm_xy_max = sqrtf(acc_mag_max_sq - sqa(lin_acc_cmd(2)));
	float p = norm_xy_max / norm_xy;
	if (p < 1.0f)
	{
		lin_acc_cmd(0) *= p;
		lin_acc_cmd(1) *= p;
	}

	// Orientation cmd
	Quat q_z(z_hat, ang_z_cmd);
	Quat q_cmd = q_z;
	float norm_acc = norm(lin_acc_cmd);
	if (norm_acc > 0)
	{
		Vector<3> acc_hat = lin_acc_cmd / norm_acc;
		float cos_z = cos(ang_z_cmd);
		float sin_z = sin(ang_z_cmd);
		float t_x = asinf(sin_z*acc_hat(0) - cos_z*acc_hat(1));
		float t_y = asinf((cos_z*acc_hat(0) + sin_z*acc_hat(1)) / cosf(t_x));
		Quat q_y(y_hat, t_y);
		Quat q_x(x_hat, t_x);
		q_cmd = q_z * q_y * q_x;
	}

	// Quaternion control
	Quat ang_err = inv(q_cmd) * ang_pos;
	if (ang_err.w < 0.0f) ang_err = -ang_err;
	Vector<3> tau_cmd;
	tau_cmd(0) = quat_x_pid.update(-ang_err.x, 0.0f, quat_sat);
	tau_cmd(1) = quat_y_pid.update(-ang_err.y, 0.0f, quat_sat);
	tau_cmd(2) = quat_z_pid.update(-ang_err.z, 0.0f, quat_sat);
	Vector<4> f_ang = D_bar * tau_cmd;

	// Acceleration z-axis cmd
	lin_acc_cmd(2) -= gravity;
	Vector<3> acc_cmd_loc = inv(ang_pos) * lin_acc_cmd;
	float acc_z_cmd = acc_cmd_loc(2);

	// Acceleration z-axis control
	float f_lin_sca = acc_z_pid.update(acc_z_cmd - lin_acc(2));
	Vector<4> f_lin;
	for (uint8_t i = 0; i < 4; i++)
	{
		f_lin(i) = f_lin_sca;
	}

	// Force regulator controller
	float p_min = 1.0f;
	quat_sat = false;
	for (uint8_t i = 0; i < 4; i++)
	{
		float p =
			(f_ang(i) > 0.0f) ? ((f_prop_max - f_lin(i)) / f_ang(i)) :
			(f_ang(i) < 0.0f) ? ((f_prop_min - f_lin(i)) / f_ang(i)) :
			1.0f;
		if ((0.0f < p) && (p < p_min))
		{
			p_min = p;
			quat_sat = true;
		}
	}
	f_props = p_min * f_ang + f_lin;
}

/**
 * @brief Resets controller to startup state
 */
void Controller::reset()
{
	quat_x_pid.reset();
	quat_y_pid.reset();
	quat_z_pid.reset();
	acc_z_pid.reset();
	quat_sat = false;
	f_props = Vector<4>();
}

/**
 * @brief Returns prop forces [N]
 */
const Vector<4>& Controller::get_f_props()
{
	return f_props;
}
