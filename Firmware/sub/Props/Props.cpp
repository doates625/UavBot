/**
 * @file Props.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Props.h"
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
namespace Props
{
	// Constants
	const float f_prop_min = 0.00f;
	const float f_prop_max = 2.46f;

	// ESC Interfaces
	const uint8_t ecs_pins[4] = { 22, 3, 23, 2 };
	ServoOut escs[4] = 
	{
		ServoOut(ecs_pins[0], f_prop_min, f_prop_max),	// M++
		ServoOut(ecs_pins[1], f_prop_min, f_prop_max),	// M+-
		ServoOut(ecs_pins[2], f_prop_min, f_prop_max),	// M-+
		ServoOut(ecs_pins[3], f_prop_min, f_prop_max),	// M--
	};

	// Forces Vector
	Vector<4> f_props;

	// Init Flag
	bool init_complete = false;
}

/**
 * @brief Inits prop motor controllers
 */
void Props::init()
{
	if (!init_complete)
	{
		// Enable ESCs
		for (uint8_t i = 0; i < 4; i++)
		{
			escs[i].set_cmd(f_prop_min);
			escs[i].set_enabled(true);
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
 */
void Props::set_f_props(const Vector<4>& f_props_)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		f_props(i) = clamp(f_props_.get(i), f_prop_min, f_prop_max);
		#if !defined(SIMULATE_PLANT)
			escs[i].set_cmd(f_props(i));
		#endif
	}
	#if defined(SIMULATE_PLANT)
		Simulator::set_f_props(f_props);
	#endif
}

/**
 * @brief Returns motor forces vector
 */
const Vector<4>& Props::get_f_props()
{
	return f_props;
}
