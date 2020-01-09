/**
 * @file State.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "State.h"
#include <Props.h>
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
			Props::set_f_props(Controller::get_f_props());
			state = Imu::is_flipped() ?
				state_failed : (state_t)Bluetooth::get_state_cmd();
			break;
		
		// Disabled state
		case state_disabled:
			Props::set_f_props(Vector<4>());
			state = (state_t)Bluetooth::get_state_cmd();
			if (state == state_enabled)
			{
				Imu::calibrate();
				Controller::reset();
			}
			break;

		// Failed state (terminal)
		case state_failed:
			Props::set_f_props(Vector<4>());
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
