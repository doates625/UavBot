/**
 * @file Motors.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Motors.h"
#if defined(SIMULATE_PLANT)
	#include <Simulator.h>
#endif
#include <Platform.h>
#include <ServoOut.h>
#include <CppUtil.h>
using CppUtil::clamp;

/**
 * Namespace Definitions
 */
namespace Motors
{
	// Constants
	const float force_min = 0.00f;
	const float force_max = 2.46f;

	// ESC Servos
	const uint8_t motor_pins[4] = { 22, 3, 23, 2 };
	ServoOut motors[4] = 
	{
		ServoOut(motor_pins[0], force_min, force_max),	// M++
		ServoOut(motor_pins[1], force_min, force_max),	// M+-
		ServoOut(motor_pins[2], force_min, force_max),	// M-+
		ServoOut(motor_pins[3], force_min, force_max),	// M--
	};

	// Forces Vector
	Vector<4> forces;

	// Init Flag
	bool init_complete = false;
}

/**
 * @brief Inits motor controllers
 */
void Motors::init()
{
	if (!init_complete)
	{
		// Enable ESCs
		for (uint8_t i = 0; i < 4; i++)
		{
			motors[i].set_cmd(force_min);
			motors[i].set_enabled(true);
		}
		Platform::wait(3.0f);

		#if defined(SIMULATE_PLANT)
			// Init simulator
			Simulator::init();
		#endif

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Sets motor forces
 * @param forces_ Motor forces vector [N]
 * 
 * Mapping:
 * - forces(0) = M++
 * - forces(1) = M+-
 * - forces(2) = M-+
 * - forces(3) = M--
 */
void Motors::set_forces(Vector<4>& forces_)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		forces(i) = clamp(forces_(i), force_min, force_max);
		#if !defined(DISABLE_MOTORS)
			motors[i].set_cmd(forces(i));
		#endif
	}
	#if defined(SIMULATE_PLANT)
		Simulator::set_forces(forces);
	#endif
}

/**
 * @brief Returns motor forces vector
 */
const Vector<4>& Motors::get_forces()
{
	return forces;
}
