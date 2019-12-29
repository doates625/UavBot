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
	void init();
	void update();
	const Quat& get_quat();
	const Vector<3>& get_omega();
	const Vector<3>& get_accel();
	float get_heading();
}
