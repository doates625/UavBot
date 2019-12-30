/**
 * @file Simulator.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Simulator.h"
#include <SerialServer.h>
#include <Struct.h>

/**
 * Namespace Definitions
 */
namespace Simulator
{
	// Serial bus settings
	usb_serial_class* const serial = &Serial;
	const uint32_t baud_rate = 115200;

	// Serial server
	const uint8_t start_byte = 0xFF;
	const uint8_t msg_id_update = 0x00;
	void msg_rx_update(uint8_t* data);
	void msg_tx_update(uint8_t* data);
	SerialServer server(serial, start_byte);
	bool got_rx = false;

	// State copies
	Quat quat; 			// Orientation [Quat]
	Vector<3> omega;	// Angular velocity [rad/s]
	Vector<3> accel;	// Local accel [m/s^2]
	Vector<4> forces;	// Propeller forces [N]

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
		server.add_rx(msg_id_update, 40, msg_rx_update);
		server.add_tx(msg_id_update, 16, msg_tx_update);

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Gets new state data from simulator
 */
void Simulator::update()
{
	while (!got_rx)
	{
		server.rx();
	}
	got_rx = false;
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
 * @brief Returns local acceleration [m/s^2]
 */
const Vector<3>& Simulator::get_accel()
{
	return accel;
}

/**
 * @brief Sends motor forces to simulator
 */
void Simulator::set_forces(const Vector<4>& forces_)
{
	forces = forces_;
	server.tx(msg_id_update);
}

/**
 * @brief Unpacks IMU data from simulator
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
 * [28-31] Local accel-x [float, m/s^2]
 * [32-35] Local accel-y [float, m/s^2]
 * [36-39] Local accel-z [float, m/s^2]
 */
void Simulator::msg_rx_update(uint8_t* data)
{
	// Make struct object
	Struct str(data);

	// Unpack simulated IMU readings
	str >> quat.w >> quat.x >> quat.y >> quat.z;
	for (uint8_t i = 0; i < 3; i++) str >> omega(i);
	for (uint8_t i = 0; i < 3; i++) str >> accel(i);

	// Set rx flag
	got_rx = true;
}

/**
 * @brief Packs force data for simulator
 * @param data Data pointer
 * 
 * Data format:
 * [00-03] Force++ [float, N]
 * [04-07] Force+- [float, N]
 * [08-11] Force-+ [float, N]
 * [12-15] Force-- [float, N]
 */
void Simulator::msg_tx_update(uint8_t* data)
{
	Struct str(data);
	for (uint8_t i = 0; i < 4; i++)
	{
		str << forces(i);
	}
}
