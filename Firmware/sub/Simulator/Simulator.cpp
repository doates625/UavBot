/**
 * @file Simulator.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Simulator.h"
#include <Motors.h>
#include <SerialServer.h>
#include <Struct.h>
using Motors::n_motors;

/**
 * Namespace Definitions
 */
namespace Simulator
{
	// Serial bus settings
	usb_serial_class* const serial = &Serial;
	const uint32_t baud_rate = 115200;

	// Serial server
	const uint8_t msg_id_data = 0x00;
	void msg_tx_data(uint8_t* data);
	void msg_rx_data(uint8_t* data);
	SerialServer server(serial);
	bool got_data = false;

	// Simulated IMU readings
	Quat quat; 			// Orientation [Quat]
	Vector<3> omega;	// Angular velocity [rad/s]
	Vector<3> accel;	// Acceleration [m/s^2]
	float heading;		// Heading [rad]

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Initializes simulator communication over USB
 */
void Simulator::init()
{
	if (!init_complete)
	{
		// Init serial bus
		serial->begin(baud_rate);

		// Configure server
		server.add_tx(msg_id_data, 16, msg_tx_data);
		server.add_rx(msg_id_data, 44, msg_rx_data);

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Gets new state data from simulator
 */
void Simulator::update()
{
	server.tx(msg_id_data);
	got_data = false;
	while (!got_data)
	{
		server.rx();
	}
}

/**
 * @brief Returns orientation [Quat]
 */
const Quat& Simulator::get_quat()
{
	return quat;
}

/**
 * @brief Returns angular velocity [rad/s]
 */
const Vector<3>& Simulator::get_omega()
{
	return omega;
}

/**
 * @brief Returns acceleration [m/s^2]
 */
const Vector<3>& Simulator::get_accel()
{
	return accel;
}

/**
 * @brief Returns heading [rad]
 */
float Simulator::get_heading()
{
	return heading;
}

/**
 * @brief Packs force command data for simulator
 * @param data Data pointer
 * 
 * Data format:
 * [00-03] Force++ [float, N]
 * [04-07] Force+- [float, N]
 * [08-11] Force-+ [float, N]
 * [12-15] Force-- [float, N]
 */
void Simulator::msg_tx_data(uint8_t* data)
{
	Vector<4> forces = Motors::get_forces();
	Struct str(data);
	for (uint8_t i = 0; i < n_motors; i++)
	{
		str << forces(i);
	}
}

/**
 * @brief Unpacks simulated IMU data from simulator
 * @param data Data pointer
 * 
 * Data format:
 * [00-03] Quat-w [float]
 * [04-07] Quat-x [float]
 * [08-11] Quat-y [float]
 * [12-15] Quat-z [float]
 * [16-19] Omega-x [float, rad/s]
 * [20-23] Omega-y [float, rad/s]
 * [24-27] Omega-z [float, rad/s]
 * [28-31] Accel-x [float, m/s^2]
 * [32-35] Accel-y [float, m/s^2]
 * [36-39] Accel-z [float, m/s^2]
 * [40-43] Heading [float, rad]
 */
void Simulator::msg_rx_data(uint8_t* data)
{
	// Make struct object
	Struct str(data);

	// Unpack simulated IMU readings
	str >> quat.w >> quat.x >> quat.y >> quat.z;
	str >> omega(0) >> omega(1) >> omega(2);
	str >> accel(0) >> accel(1) >> accel(2);
	str >> heading;

	// Set got data flag
	got_data = true;
}
