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
#include <Timer.h>
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
	const uint8_t msg_id_state = 0xAA;
	const uint8_t msg_id_params = 0xBB;
	const uint8_t msg_id_update = 0xCC;
	void msg_rx_state(uint8_t* data);
	void msg_rx_params(uint8_t* data);
	void msg_rx_update(uint8_t* data);
	void msg_tx_update(uint8_t* data);
	SerialServer server(serial, start_byte);

	// Flight commands
	uint8_t state_cmd = State::state_disabled;
	Quat ang_pos_cmd;
	float thr_lin_cmd;
	bool got_cmds = false;

	// Flight params
	bool new_params = false;
	Params params;
	
	// Command timer [s]
	Timer cmd_timer;

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
		server.add_rx(msg_id_params, 56, msg_rx_params);
		server.add_rx(msg_id_update, 20, msg_rx_update);
		server.add_tx(msg_id_update, 57, msg_tx_update);

		// Start command timer
		cmd_timer.start();

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
	if (got_cmds)
	{
		cmd_timer.reset();
		got_cmds = false;
	}
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
 * @brief Returns orientation cmd [Quat]
 */
const Quat& Bluetooth::get_ang_pos_cmd()
{
	return ang_pos_cmd;
}

/**
 * @brief Returns linear throttle cmd [0, 1]
 */
float Bluetooth::get_thr_lin_cmd()
{
	return thr_lin_cmd;
}

/**
 * @brief Returns true if new params have been received since last call
 */
bool Bluetooth::got_new_params()
{
	bool new_params_copy = new_params;
	new_params = false;
	return new_params_copy;
}

/**
 * @brief Returns flight params
 */
const Params& Bluetooth::get_params()
{
	return params;
}

/**
 * @brief Reset command timer
 */
void Bluetooth::reset_cmd_timer()
{
	cmd_timer.reset();
}

/**
 * @brief Returns time since last cmd received [s]
 */
float Bluetooth::get_cmd_time()
{
	return cmd_timer.read();
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
 * @brief Gets UAV flight parameters
 * @param data Data pointer
 * 
 * Data format:
 * - Min linear throttle [float, [0, 1]]
 * - Max linear throttle [float, [0, 1]]
 * - Quat-x P-gain [thr/rad]
 * - Quat-x I-gain [thr/(rad*s)]
 * - Quat-x D-gain [thr/(rad/s)]
 * - Quat-x feed-forward [thr]
 * - Quat-y P-gain [thr/rad]
 * - Quat-y I-gain [thr/(rad*s)]
 * - Quat-y D-gain [thr/(rad/s)]
 * - Quat-y feed-forward [thr]
 * - Quat-z P-gain [thr/rad]
 * - Quat-z I-gain [thr/(rad*s)]
 * - Quat-z D-gain [thr/(rad/s)]
 * - Quat-z feed-forward [thr]
 */
void Bluetooth::msg_rx_params(uint8_t* data)
{
	Struct str(data);
	Platform::disable_interrupts();
	str >> params.thr_min;
	str >> params.thr_max;
	str >> params.qx_kp;
	str >> params.qx_ki;
	str >> params.qx_kd;
	str >> params.qx_ff;
	str >> params.qy_kp;
	str >> params.qy_ki;
	str >> params.qy_kd;
	str >> params.qy_ff;
	str >> params.qz_kp;
	str >> params.qz_ki;
	str >> params.qz_kd;
	str >> params.qz_ff;
	Platform::enable_interrupts();
	new_params = true;
}

/**
 * @brief Gets teleop commands and sends state back
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
