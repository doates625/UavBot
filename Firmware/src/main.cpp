/**
 * @file main.cpp
 * @brief Main functions for UavBot firmware
 * @author Dan Oates (WPI Class of 2020)
 */
#include <Imu.h>
#include <Controller.h>
#include <Motors.h>
#include <Bluetooth.h>
#include <DigitalOut.h>
using Controller::t_ctrl_us;

/**
 * Global Variables
 */
const uint8_t pin_led = 13;
DigitalOut led(pin_led);
IntervalTimer timer;

/**
 * @brief Runs control loop once
 */
void run_ctrl()
{
	led = 1;
	Imu::update();
	Controller::update();
	led = 0;
}

/**
 * @brief Arduino setup
 */
void setup()
{
	// Init subsystems
	led = 1;
	Imu::init();
	Motors::init();
	Controller::init();
	Bluetooth::init();
	led = 0;

	// Init control interrupt
	timer.begin(run_ctrl, t_ctrl_us);
}

/**
 * @brief Arduino loop
 */
void loop()
{
	Bluetooth::update();
}
