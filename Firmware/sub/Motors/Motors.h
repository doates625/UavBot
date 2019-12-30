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
	extern const float force_min;	// Min prop force [N]
	extern const float force_max;	// Max prop force [N]

	// Functions
	void init();
	void set_forces(Vector<4>& forces_);
	void set_forces(float force);
	const Vector<4>& get_forces();
}
