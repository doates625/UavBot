/**
 * @file State.h
 * @brief Subsystem for UAV state machine
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once

/**
 * Namespace Declaration
 */
namespace State
{
	// State enum
	typedef enum
	{
		state_enabled,
		state_disabled,
		state_failure,
	}
	state_t;

	// Function
	void update();
	state_t get();
}
