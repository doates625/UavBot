/**
 * @file Imu.h
 * @brief Subsystem for UAV BNO055 IMU interface
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <QuatCpp.h>
#include <LinearCpp.h>

/**
 * @brief Namespace Declaration
 */
namespace Imu
{
	bool init();
	void calibrate();
	void update();
	const Quat& get_ang_pos();
	const Vector<3>& get_ang_vel();
	const Vector<3>& get_lin_acc();
	bool is_flipped();
}
