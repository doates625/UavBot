/**
 * @file DebugLed.h
 * @brief Subsystem for debug LED
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <DigitalOut.h>

/**
 * Namespace Declaration
 */
namespace DebugLed
{
	void set(int on);
	void flash(uint8_t n);
}
