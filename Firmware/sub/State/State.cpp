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
			Controller::update();
			Motors::set_forces(Controller::get_forces());
			state = Imu::is_flipped() ?
				state_failed : (state_t)Bluetooth::get_state_cmd();
			break;
		
		// Disabled state
		case state_disabled:
			Motors::set_forces(Vector<4>());
			state = (state_t)Bluetooth::get_state_cmd();
			if (state == state_enabled)
			{
				Controller::reset();
			}
			break;

		// Failed state (terminal)
		case state_failed:
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
