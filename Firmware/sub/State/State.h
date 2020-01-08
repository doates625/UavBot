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
		state_enabled = 0x00,
		state_disabled = 0x01,
		state_failed = 0x02,
	}
	state_t;

	// Function
	void update();
	state_t get();
}
