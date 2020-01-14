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
	const uint8_t msg_id_update = 0xAA;
	void msg_rx_update(uint8_t* data);
	void msg_tx_update(uint8_t* data);
	SerialServer server(serial, start_byte);
	bool got_rx = false;

	// State copies
	Quat ang_pos; 			// Orientation [Quat]
	Vector<3> ang_vel;		// Angular velocity [rad/s]
	Vector<3> lin_acc;		// Linear accel [m/s^2]
	Vector<4> thr_props;	// Prop throttles

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
const Quat& Simulator::get_ang_pos()
{
	return ang_pos;
}

/**
 * @brief Returns angular velocity [rad/s]
 */
const Vector<3>& Simulator::get_ang_vel()
{
	return ang_vel;
}

/**
 * @brief Returns local linear acceleration [m/s^2]
 */
const Vector<3>& Simulator::get_lin_acc()
{
	return lin_acc;
}

/**
 * @brief Sends prop throttles to simulator
 */
void Simulator::set_thr_props(const Vector<4>& thr)
{
	thr_props = thr;
	server.tx(msg_id_update);
}

/**
 * @brief Unpacks IMU data from simulator
 * @param data Data pointer
 * 
 * Data format:
 * - Angular position [float, [w; x; y; z]]
 * - Angular velocity [float, [x; y; z], rad/s]
 * - Local acceleration [float, [x; y; z], m/s^2]
 */
void Simulator::msg_rx_update(uint8_t* data)
{
	// Make struct object
	Struct str(data);

	// Unpack simulated IMU readings
	str >> ang_pos.w >> ang_pos.x >> ang_pos.y >> ang_pos.z;
	for (uint8_t i = 0; i < 3; i++) str >> ang_vel(i);
	for (uint8_t i = 0; i < 3; i++) str >> lin_acc(i);

	// Set rx flag
	got_rx = true;
}

/**
 * @brief Packs force data for simulator
 * @param data Data pointer
 * 
 * Data format:
 * - Prop throttles [float, [++, +-, -+, --], [0, 1]]
 */
void Simulator::msg_tx_update(uint8_t* data)
{
	Struct str(data);
	for (uint8_t i = 0; i < 4; i++)
	{
		str << thr_props(i);
	}
}
