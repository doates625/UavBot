/**
 * @file Props.h
 * @brief Subsystem for UAV propeller motor controllers
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <LinearCpp.h>

/**
 * Namespace Declaration
 */
namespace Props
{
	// Constants
	extern const float f_prop_min;	// Min prop force [N]
	extern const float f_prop_max;	// Max prop force [N]

	// Functions
	void init();
	void set_f_props(const Vector<4>& f_props_);
	const Vector<4>& get_f_props();
}
