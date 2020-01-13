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
	// ESC interfaces
	const uint8_t ecs_pins[4] = { 22, 3, 23, 2 };
	ServoOut escs[4] = 
	{
		ServoOut(ecs_pins[0], 0.0f, 1.0f),	// Motor [++]
		ServoOut(ecs_pins[1], 0.0f, 1.0f),	// Motor [+-]
		ServoOut(ecs_pins[2], 0.0f, 1.0f),	// Motor [-+]
		ServoOut(ecs_pins[3], 0.0f, 1.0f),	// Motor [--]
	};

	// Throttle vctor
	Vector<4> thr_props;

	// Init flag
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
			escs[i].set_cmd(0.0f);
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
 * @brief Sets prop throttles
 * @param thr Prop throttles vector [0, 1]
 */
void Props::set_thr(const Vector<4>& thr)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		thr_props(i) = clamp(thr.get(i), 0.0f, 1.0f);
		#if !defined(SIMULATE_PLANT)
			escs[i].set_cmd(thr_props(i));
		#endif
	}
	#if defined(SIMULATE_PLANT)
		Simulator::set_thr_props(thr_props);
	#endif
}

/**
 * @brief Returns prop throttles vector [0, 1]
 */
const Vector<4>& Props::get_thr()
{
	return thr_props;
}
