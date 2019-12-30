/**
 * @file DebugLed.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "DebugLed.h"
#include <Platform.h>

/**
 * Namespace Definitions
 */
namespace DebugLed
{
	const uint8_t pin_led = 13;
	DigitalOut led(pin_led);
}

/**
 * @brief Turns led on or off
 * @param on 1 = on, 0 = off
 */
void DebugLed::set(int on)
{
	led = on;
}

/**
 * @brief Flashes LED n times in infinite loop
 */
void DebugLed::flash(uint8_t n)
{
	while (true)
	{
		for (uint8_t i = 0; i < n; i++)
		{
			set(1);
			Platform::wait(0.10f);
			set(0);
			Platform::wait(0.15f);
		}
		Platform::wait(1.0f);
	}
}
