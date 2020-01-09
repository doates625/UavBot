/**
 * @file Imu.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Imu.h"
#if !defined(SIMULATE_PLANT)
	#include <Platform.h>
	#include <BNO055.h>
#else
	#include <Simulator.h>
#endif
#include <CppUtil.h>
using CppUtil::sqa;

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
	const BNO055::axis_config_t axis_config = BNO055::NWU;
	BNO055 bno055(wire, axis_config);

	// Quat calibration offset
	Quat quat_cal;
#endif

	// IMU readings
	Quat quat; 			// Orientation [Quat]
	Vector<3> omega;	// Angular velocity [rad/s]
	Vector<3> accel;	// Acceleration (with gravity) [m/s^2]
	
	// Init flag
	bool init_complete = false;
}

/**
 * @brief Initializes IMU and waits for calibration
 * @return True if connection was successful
 */
bool Imu::init()
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
			Platform::wait(0.1);
			if (!bno055.init())
			{
				return false;
			}
			while (!bno055.calibrated());
		#else
			// Init simulator
			Simulator::init();
		#endif

		// Set init flag
		init_complete = true;
	}
	return true;
}

/**
 * @brief Calibrates orientation to unity
 */
void Imu::calibrate()
{
	quat_cal = Quat();
	update();
	quat_cal = inv(quat);
}

/**
 * @brief Updates IMU readings
 */
void Imu::update()
{
#if defined(SIMULATE_PLANT)

	// Get readings from simulator
	Simulator::update();
	quat = Simulator::get_quat();
	omega = Simulator::get_omega();
	accel = Simulator::get_accel();

#elif !defined(STUB_IMU)

	// Update quat
	bno055.update_qua();
	quat.w = bno055.get_qua_w();
	quat.x = bno055.get_qua_x();
	quat.y = bno055.get_qua_y();
	quat.z = bno055.get_qua_z();
	quat = quat_cal * quat;
	
	// Update omega
	bno055.update_gyr();
	omega(0) = bno055.get_gyr_x();
	omega(1) = bno055.get_gyr_y();
	omega(2) = bno055.get_gyr_z();
	
	// Update accel
	bno055.update_lia();
	accel(0) = bno055.get_lia_x();
	accel(1) = bno055.get_lia_y();
	accel(2) = bno055.get_lia_z();
	
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
 * @brief Returns true if UAV is flipped upside down
 */
bool Imu::is_flipped()
{
	return (sqa(quat.x) + sqa(quat.y)) > 0.5f;
}
