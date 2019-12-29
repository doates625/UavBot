/**
 * @file Simulator.h
 * @brief Subsystem for Matlab plant simulator
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <QuatCpp.h>
#include <LinearCpp.h>

/**
 * Namespace Declaration
 */
namespace Simulator
{
	void init();
	void update();
	const Quat& get_quat();
	const Vector<3>& get_omega();
	const Vector<3>& get_accel();
	float get_heading();
}
