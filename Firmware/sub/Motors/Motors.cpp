/**
 * @file Motors.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Motors.h"
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
	const uint8_t n_motors = 4;

	// ESC Servos
	const uint8_t motor_pins[n_motors] = { 22, 3, 23, 2 };
	ServoOut motors[n_motors] = 
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
		// Set forces to zero
		for (uint8_t i = 0; i < n_motors; i++)
		{
			motors[i].set_cmd(0.0f);
		}

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
	for (uint8_t i = 0; i < n_motors; i++)
	{
		forces(i) = clamp(forces_(i), force_min, force_max);
		#if defined(ENABLE_MOTORS)
			motors[i].set_cmd(forces(i));
		#endif
	}
}

/**
 * @brief Sets all motor forces to same force
 * @param force Motor force [N]
 */
void Motors::set_forces(float force)
{
	force = clamp(force, force_min, force_max);
	for (uint8_t i = 0; i < n_motors; i++)
	{
		forces(i) = force;
		#if defined(ENABLE_MOTORS)
			motors[i].set_cmd(force);
		#endif
	}
}

/**
 * @brief Returns motor forces vector
 */
const Vector<4>& Motors::get_forces()
{
	return forces;
}
