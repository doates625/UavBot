/**
 * @file Bluetooth.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Bluetooth.h"
#include <Imu.h>
#include <Props.h>
#include <State.h>
#include <Platform.h>
#include <SerialServer.h>
#include <Struct.h>
#include <QuatCpp.h>
using State::state_t;

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
	const uint8_t msg_id_state = 0x00;
	const uint8_t msg_id_update = 0x01;
	void msg_rx_state(uint8_t* data);
	void msg_rx_update(uint8_t* data);
	void msg_tx_update(uint8_t* data);
	SerialServer server(serial, start_byte);

	// Controller commands
	uint8_t state_cmd = State::state_disabled;
	Quat ang_pos_cmd;
	float thr_lin_cmd;
	bool got_cmds = false;	

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
		server.add_rx(msg_id_state, 1, msg_rx_state);
		server.add_rx(msg_id_update, 20, msg_rx_update);
		server.add_tx(msg_id_update, 57, msg_tx_update);

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
	bool got_cmds_copy = got_cmds;
	got_cmds = false;
	return got_cmds_copy;
}

/**
 * @brief Returns state command as byte
 */
uint8_t Bluetooth::get_state_cmd()
{
	return state_cmd;
}

/**
 * @brief Returns orientation cmd
 */
const Quat& Bluetooth::get_ang_pos_cmd()
{
	return ang_pos_cmd;
}

/**
 * @brief 
 */
float Bluetooth::get_thr_lin_cmd()
{
	return thr_lin_cmd;
}

/**
 * @brief Gets UAV state command
 * @param data Data pointer
 * 
 * Data format:
 * - State command [uint8_t]
 */
void Bluetooth::msg_rx_state(uint8_t* data)
{
	state_cmd = data[0];
}

/**
 * @brief Gets teleop commands from remote and sends state back
 * @param data Data pointer
 * 
 * Data format:
 * - Angular position cmd [float, [w; x; y; z]]
 * - Linear throttle cmd [float]
 */
void Bluetooth::msg_rx_update(uint8_t* data)
{
	// Copy cmds with ISRs disabled
	Struct str(data);
	Platform::disable_interrupts();
	str >> ang_pos_cmd.w;
	str >> ang_pos_cmd.x;
	str >> ang_pos_cmd.y;
	str >> ang_pos_cmd.z;
	str >> thr_lin_cmd;
	Platform::enable_interrupts();

	// Transmit state response
	server.tx(msg_id_update);
	got_cmds = true;
}

/**
 * @brief Transmits state data to remote
 * @param data Data pointer
 * 
 * Data format:
 * - Angular position [float, [w; x; y; z]]
 * - Angular velocity [float, [x; y; z], rad/s]
 * - Local acceleration [float, [x; y; z], m/s^2]
 * - Prop throttles [float, [++, +-, -+, --], [0, 1]]
 * - State command [uint8]
 */
void Bluetooth::msg_tx_update(uint8_t* data)
{
	// Copy state data with ISRs disabled
	Platform::disable_interrupts();
	Vector<4> ang_pos = Imu::get_ang_pos();
	Vector<3> ang_vel = Imu::get_ang_vel();
	Vector<3> lin_acc = Imu::get_lin_acc();
	Vector<4> thr_props = Props::get_thr();
	state_t state = State::get();
	Platform::enable_interrupts();

	// Pack data
	Struct str(data);
	for (uint8_t i = 0; i < 4; i++) str << ang_pos(i);
	for (uint8_t i = 0; i < 3; i++) str << ang_vel(i);
	for (uint8_t i = 0; i < 3; i++) str << lin_acc(i);
	for (uint8_t i = 0; i < 4; i++) str << thr_props(i);
	str << (uint8_t)state;
}
