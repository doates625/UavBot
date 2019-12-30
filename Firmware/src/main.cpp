/**
 * @file main.cpp
 * @brief Main functions for UavBot firmware
 * @author Dan Oates (WPI Class of 2020)
 */
#include <Imu.h>
#include <Controller.h>
#include <Motors.h>
#include <Bluetooth.h>
#include <Platform.h>
#include <DigitalOut.h>
using Controller::t_ctrl_us;
using Platform::wait;

/**
 * Global Variables
 */
const uint8_t pin_led = 13;
DigitalOut led(pin_led);
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
	led = 1;
	if (!Imu::init()) error(1);
	Motors::init();
	Controller::init();
	Bluetooth::init();
	led = 0;

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
	led = 1;
	Imu::update();
	#if defined(SIMULATE_PLANT)
		Bluetooth::update();
	#endif
	Controller::update();
	led = 0;
}

/**
 * @brief Disables motors and flashes LED n times in a loop
 */
void error(uint8_t n)
{
	Motors::set_forces(0.0f);
	while (true)
	{
		for (uint8_t i = 0; i < n; i++)
		{
			led = 1;
			Platform::wait(0.10f);
			led = 0;
			Platform::wait(0.15f);
		}
		Platform::wait(1.0f);
	}
}
