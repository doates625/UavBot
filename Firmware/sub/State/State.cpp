/**
 * @file State.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "State.h"
#include <Motors.h>
#include <Controller.h>
#include <Imu.h>
#include <Bluetooth.h>
#include <DebugLed.h>

/**
 * Namespace Definitions
 */
namespace State
{
	// Begin disabled for safety
	state_t state = state_disabled;
}

/**
 * @brief Updates state machine
 */
void State::update()
{
	switch (state)
	{
		// Enabled state
		case state_enabled:
			Motors::set_forces(Controller::get_forces());
			if (Imu::is_flipped())
			{
				state = state_failure;
			}
			else if (!Bluetooth::get_enable_cmd())
			{
				state = state_disabled;
			}
			break;
		
		// Disabled state
		case state_disabled:
			Motors::set_forces(Vector<4>());
			if (Bluetooth::get_enable_cmd())
			{
				state = state_enabled;
			}
			break;

		// Failure state (terminal)
		case state_failure:
			Motors::set_forces(Vector<4>());
			break;
	}
}

/**
 * @brief Returns state
 */
State::state_t State::get()
{
	return state;
}
