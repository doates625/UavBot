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
	const Quat& get_ang_pos();
	const Vector<3>& get_ang_vel();
	const Vector<3>& get_lin_acc();
	void set_thr_props(const Vector<4>& thr);
}
