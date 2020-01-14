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
	state_t state = state_disabled;	// State enum
	const float cmd_timeout = 0.5f;	// Cmd timeout [s]
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
		{
			Controller::update();
			Props::set_thr(Controller::get_thr_props());
			bool timed_out = Bluetooth::get_cmd_time() > cmd_timeout;
			if (timed_out || Imu::is_flipped())
			{
				state = state_failed;
			}
			else
			{
				state = (state_t)Bluetooth::get_state_cmd();
			}
		}
		break;
		
		// Disabled state
		case state_disabled:
		{
			Props::set_thr(Vector<4>());
			if (Bluetooth::got_new_params())
			{
				Controller::set_params(Bluetooth::get_params());
			}
			state = (state_t)Bluetooth::get_state_cmd();
			if (state == state_enabled)
			{
				Imu::calibrate();
				Controller::reset();
				Bluetooth::reset_cmd_timer();
			}
		}
		break;

		// Failed state (terminal)
		case state_failed:
		{
			Props::set_thr(Vector<4>());
		}
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
