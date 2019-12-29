/**
 * @file Bluetooth.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Bluetooth.h"
#include <Imu.h>
#include <Platform.h>
#include <SerialServer.h>
#include <Struct.h>
#include <QuatCpp.h>

/**
 * Namespace Definitions
 */
namespace Bluetooth
{
	// Serial bus settings
	HardwareSerial* const serial = &Serial3;
	const uint32_t baud_rate = 57600;

	// Serial server
	const uint8_t msg_id_start = 0x00;
	const uint8_t msg_id_teleop = 0x01;
	const uint8_t msg_id_state = 0x02;
	void msg_rx_start(uint8_t* data);
	void msg_rx_teleop(uint8_t* data);
	void msg_tx_state(uint8_t* data);
	SerialServer server(serial);

	// Controller commands
	bool start_cmd = false;
	Vector<3> acc_cmd;
	float heading_cmd = 0.0f;

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Inits Bluetooth and waits for start command
 */
void Bluetooth::init()
{
	if (!init_complete)
	{
		// Init serial
		serial->begin(baud_rate);

		// Configure server
		server.add_rx(msg_id_start, 0, msg_rx_start);
		server.add_rx(msg_id_teleop, 16, msg_rx_teleop);
		server.add_tx(msg_id_state, 44, msg_tx_state);

		// Wait for start command
		while (!start_cmd)
		{
			update();
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Processes all serial messages
 */
void Bluetooth::update()
{
	server.rx();
}

/**
 * @brief Returns global accel cmd [m/s^2]
 */
const Vector<3>& Bluetooth::get_acc_cmd()
{
	return acc_cmd;
}

/**
 * @brief Returns heading cmd [rad]
 */
float Bluetooth::get_heading_cmd()
{
	return heading_cmd;
}

/**
 * @brief Tells UAV to start flight control
 * @param data Data pointer (empty)
 */
void Bluetooth::msg_rx_start(uint8_t* data)
{
	start_cmd = true;
}

/**
 * @brief Unpacks teleop commands from remote and sends state back
 * @param data Data pointer
 * 
 * Data format:
 * [00-03] Global accel x [float, m/s^2]
 * [04-07] Global accel y [float, m/s^2]
 * [08-11] Global accel z [float, m/s^2]
 * [12-15] Heading cmd [float, rad]
 */
void Bluetooth::msg_rx_teleop(uint8_t* data)
{
	Struct str(data);
	str >> acc_cmd(0);
	str >> acc_cmd(1);
	str >> acc_cmd(2);
	str >> heading_cmd;
	server.tx(msg_id_state);
}

/**
 * @brief Transmits state data to remote
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
void Bluetooth::msg_tx_state(uint8_t* data)
{
	// Copy state data with ISRs disabled
	Platform::disable_interrupts();
	Quat quat = Imu::get_quat();
	Vector<3> omega = Imu::get_omega();
	Vector<3> accel = Imu::get_accel();
	float heading = Imu::get_heading();
	Platform::enable_interrupts();

	// Pack state data
	Struct str(data);
	str << quat.w << quat.x << quat.y << quat.z;
	str << omega(0) << omega(1) << omega(2);
	str << accel(1) << accel(2) << accel(3);
	str << heading;
}
