/**
 * @file Motors.h
 * @brief Subsystem for UAV motor drivers
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <LinearCpp.h>

/**
 * Namespace Declaration
 */
namespace Motors
{
	// Constants
	extern const float f_prop_min;	// Min prop force [N]
	extern const float f_prop_max;	// Max prop force [N]

	// Functions
	void init();
	void set_forces(const Vector<4>& forces_);
	const Vector<4>& get_forces();
}
