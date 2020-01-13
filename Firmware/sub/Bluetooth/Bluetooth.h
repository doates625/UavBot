/**
 * @file Bluetooth.h
 * @brief Subsystem for Bluetooth remote control
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <QuatCpp.h>

/**
 * Namespace Declaration
 */
namespace Bluetooth
{
	void init();
	bool update();
	uint8_t get_state_cmd();
	const Quat& get_ang_pos_cmd();
	float get_thr_lin_cmd();
}
