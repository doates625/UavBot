/**
 * @file main.cpp
 * @brief Main functions for UavBot firmware
 * @author Dan Oates (WPI Class of 2020)
 */
#include <Imu.h>
#include <Controller.h>
#include <Motors.h>
#include <Bluetooth.h>
#include <State.h>
#include <DebugLed.h>
#include <Platform.h>
#include <DigitalOut.h>
using Controller::t_ctrl_us;
using State::state_enabled;
using Platform::wait;

/**
 * Global Variables
 */
IntervalTimer timer;

/**
 * Function Templates
 */
void run_ctrl();
void error(uint8_t n);

/**
 * @brief Arduino setup
 */
void setup()
{
	// Init subsystems
	DebugLed::set(1);
	if (!Imu::init())
	{
		DebugLed::flash(1);
	}
	Motors::init();
	Controller::init();
	Bluetooth::init();
	DebugLed::set(0);

	// Init control interrupt
	#if !defined(SIMULATE_PLANT)
		timer.begin(run_ctrl, t_ctrl_us);
	#endif
}

/**
 * @brief Arduino loop
 */
void loop()
{
	#if !defined(SIMULATE_PLANT)
		Bluetooth::update();
	#else
		run_ctrl();
	#endif
}

/**
 * @brief Runs control loop once
 */
void run_ctrl()
{
	// Start loop
	DebugLed::set(1);

	#if defined(SIMULATE_PLANT)
		// Get simulator commands
		while (!Bluetooth::update());
	#endif

	// Control system
	Imu::update();
	State::update();
	
	// End loop
	DebugLed::set(0);
}
