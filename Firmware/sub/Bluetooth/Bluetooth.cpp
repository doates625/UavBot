/**
 * @file Bluetooth.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Bluetooth.h"
#include <Imu.h>
#include <Motors.h>
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
	const uint8_t start_byte = 0xFF;
	const uint8_t msg_id_start = 0x00;
	const uint8_t msg_id_update = 0x01;
	void msg_rx_start(uint8_t* data);
	void msg_rx_update(uint8_t* data);
	void msg_tx_update(uint8_t* data);
	SerialServer server(serial, start_byte);

	// Controller commands
	bool start_cmd = false;
	bool update_cmd = false;
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
		server.add_rx(msg_id_update, 16, msg_rx_update);
		server.add_tx(msg_id_update, 56, msg_tx_update);

		// Wait for start command
		#if !defined(STUB_BLUETOOTH)
			while (!start_cmd)
			{
				update();
			}
		#endif

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Processes all serial messages
 * @return True if update command was received
 */
bool Bluetooth::update()
{
	server.rx();
	bool ret = update_cmd;
	update_cmd = false;
	return ret;
}

/**
 * @brief Returns global accel cmd [m/s^2]
 */
const Vector<3>& Bluetooth::get_acc_cmd()
{
	#if defined(STUB_BLUETOOTH)
		acc_cmd(0) = 0.0f;
		acc_cmd(1) = 0.0f;
		acc_cmd(2) = -10.0f;
	#endif
	return acc_cmd;
}

/**
 * @brief Returns heading cmd [rad]
 */
float Bluetooth::get_heading_cmd()
{
	#if defined(STUB_BLUETOOTH)
		heading_cmd = 0.0f;
	#endif
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
 * @brief Gets teleop commands from remote and sends state back
 * @param data Data pointer
 * 
 * Data format:
 * [00-03] Global accel x [float, m/s^2]
 * [04-07] Global accel y [float, m/s^2]
 * [08-11] Global accel z [float, m/s^2]
 * [12-15] Heading cmd [float, rad]
 */
void Bluetooth::msg_rx_update(uint8_t* data)
{
	Struct str(data);
	for (uint8_t i = 0; i < 3; i++) str >> acc_cmd(i);
	str >> heading_cmd;
	server.tx(msg_id_update);
	update_cmd = true;
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
 * [40-43] Force++ [float, N]
 * [44-47] Force+- [float, N]
 * [48-51] Force-+ [float, N]
 * [52-55] Force-- [float, N]
 */ 
void Bluetooth::msg_tx_update(uint8_t* data)
{
	// Copy state data with ISRs disabled
	Platform::disable_interrupts();
	Vector<4> quat = Imu::get_quat();
	Vector<3> omega = Imu::get_omega();
	Vector<3> accel = Imu::get_accel();
	Vector<4> forces = Motors::get_forces();
	Platform::enable_interrupts();

	// Pack state data
	Struct str(data);
	for (uint8_t i = 0; i < 4; i++) str << quat(i);
	for (uint8_t i = 0; i < 3; i++) str << omega(i);
	for (uint8_t i = 0; i < 3; i++) str << accel(i);
	for (uint8_t i = 0; i < 4; i++) str << forces(i);
}
