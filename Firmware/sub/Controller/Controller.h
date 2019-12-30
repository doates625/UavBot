/**
 * @file Controller.h
 * @brief Subsystem for UAV flight controller
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once

/**
 * Namespace Declaration
 */
namespace Controller
{
	// Constants
	extern const float f_ctrl;		// Control freq [Hz]
	extern const float t_ctrl_us;	// Control period [us]

	// Functions
	void init();
	void update();
}