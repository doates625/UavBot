/**
 * @file Imu.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Imu.h"
#if !defined(SIMULATE_PLANT)
	#include <BNO055.h>
#else
	#include <Simulator.h>
#endif

/**
 * Namespace Definitions
 */
namespace Imu
{
#if !defined(SIMULATE_PLANT)
	// I2C bus settings
	const uint8_t pin_sda = 18;
	const uint8_t pin_scl = 19;
	const uint32_t bit_rate = 400000;
	TwoWire* const wire = &Wire;

	// BNO055 IMU
	const BNO055::axis_config_t axis_config = BNO055::tlf;
	BNO055 bno055(wire, axis_config);
#endif

	// IMU readings
	Quat quat; 			// Orientation [Quat]
	Vector<3> omega;	// Angular velocity [rad/s]
	Vector<3> accel;	// Acceleration [m/s^2]
	float heading;		// Heading [rad]
	
	// Init flag
	bool init_complete = false;
}

/**
 * @brief Initializes IMU
 */
void Imu::init()
{
	if (!init_complete)
	{
		#if !defined(SIMULATE_PLANT)
			// Init I2C bus
			wire->begin();
			wire->setClock(bit_rate);
			wire->setSDA(pin_sda);
			wire->setSCL(pin_scl);

			// Init BNO055
			bno055.init();
			// TODO check for compass lock

			// Init readings
			heading = 0.0f;
		#else
			// Init simulator
			Simulator::init();
		#endif

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Updates IMU readings
 */
void Imu::update()
{
#if !defined(SIMULATE_PLANT)
	// Update readings
	bno055.update_qua();
	bno055.update_gyr();
	bno055.update_lia();

	// Copy quat
	quat.w = bno055.get_qua_w();
	quat.x = bno055.get_qua_x();
	quat.y = bno055.get_qua_y();
	quat.z = bno055.get_qua_z();
	
	// Copy omega
	omega(0) = bno055.get_gyr_x();
	omega(1) = bno055.get_gyr_y();
	omega(2) = bno055.get_gyr_z();
	
	// Copy accel
	accel(0) = bno055.get_lia_x();
	accel(1) = bno055.get_lia_y();
	accel(2) = bno055.get_lia_z();

	// Copy heading
	heading = bno055.get_eul_h();
#else
	// Get readings from simulator
	Simulator::update();
	quat = Simulator::get_quat();
	omega = Simulator::get_omega();
	accel = Simulator::get_accel();
	heading = Simulator::get_heading();
#endif
}

/**
 * @brief Returns orientation [Quat]
 */
const Quat& Imu::get_quat()
{
	return quat;
}

/**
 * @brief Returns angular velocity [rad/s]
 */
const Vector<3>& Imu::get_omega()
{
	return omega;
}

/**
 * @brief Returns acceleration [m/s^2]
 */
const Vector<3>& Imu::get_accel()
{
	return accel;
}

/**
 * @brief Returns heading [rad]
 */
float Imu::get_heading()
{
	return heading;
}
