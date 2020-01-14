/**
 * @file Controller.h
 * @brief Subsystem for UAV flight controller
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <LinearCpp.h>
#include <Params.h>

/**
 * Namespace Declaration
 */
namespace Controller
{
	// Timing constants
	extern const float f_ctrl_hz;	// Control freq [Hz]
	extern const float t_ctrl_us;	// Control period [us]

	// Functions
	void init();
	void set_params(const Params& params);
	void update();
	void reset();
	const Vector<4>& get_thr_props();
}
