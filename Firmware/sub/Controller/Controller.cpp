/**
 * @file Controller.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Controller.h"
#include <Imu.h>
#include <Motors.h>

/**
 * Namespace Definitions
 */
namespace Controller
{
	// Constants
	const float f_ctrl = 50.0f;
	const float t_ctrl_us = 1e6f / f_ctrl;

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Inits controller
 */
void Controller::init()
{
	if (!init_complete)
	{
		// TODO initialize matrices

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Runs one control loop iteration
 */
void Controller::update()
{
	// TODO (this is fake - DUH)
	Vector<4> forces;
	forces(0) = +1.1f;
	forces(2) = +1.0f;
	forces(1) = +1.0f;
	forces(3) = +1.1f;
	Motors::set_forces(forces);
}