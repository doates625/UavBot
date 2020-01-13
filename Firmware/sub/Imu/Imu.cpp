/**
 * @file Imu.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Imu.h"
#include <Platform.h>
#include <BNO055.h>
#if defined(SIMULATE_PLANT)
	#include <Simulator.h>
#endif
#include <CppUtil.h>
using CppUtil::sqa;

/**
 * Namespace Definitions
 */
namespace Imu
{
	// I2C bus settings
	const uint8_t pin_sda = 18;
	const uint8_t pin_scl = 19;
	const uint32_t bit_rate = 400000;
	TwoWire* const wire = &Wire;

	// BNO055 IMU
	const BNO055::axis_config_t axis_config = BNO055::NWU;
	BNO055 bno055(wire, axis_config);

	// Quat calibration offset
	Quat ang_pos_cal;

	// IMU readings
	Quat ang_pos; 		// Angular position [Quat]
	Vector<3> ang_vel;	// Angular velocity [rad/s]
	Vector<3> lin_acc;	// Local linear acceleration [m/s^2]
	
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
#if !defined(SIMULATE_PLANT)
	ang_pos_cal = Quat();
	update();
	ang_pos_cal = inv(ang_pos);
#endif
}

/**
 * @brief Updates IMU readings
 */
void Imu::update()
{
#if defined(SIMULATE_PLANT)

	// Get readings from simulator
	Simulator::update();
	ang_pos = Simulator::get_ang_pos();
	ang_vel = Simulator::get_ang_vel();
	lin_acc = Simulator::get_lin_acc();

#elif !defined(STUB_IMU)

	// Update quat
	bno055.update_qua();
	ang_pos.w = bno055.get_qua_w();
	ang_pos.x = bno055.get_qua_x();
	ang_pos.y = bno055.get_qua_y();
	ang_pos.z = bno055.get_qua_z();
	ang_pos = ang_pos_cal * ang_pos;
	if (ang_pos.w < 0.0f) ang_pos = -ang_pos;
	
	// Update omega
	bno055.update_gyr();
	ang_vel(0) = bno055.get_gyr_x();
	ang_vel(1) = bno055.get_gyr_y();
	ang_vel(2) = bno055.get_gyr_z();
	
	// Update accel
	bno055.update_lia();
	lin_acc(0) = bno055.get_lia_x();
	lin_acc(1) = bno055.get_lia_y();
	lin_acc(2) = bno055.get_lia_z();
	
#endif
}

/**
 * @brief Returns orientation [Quat]
 */
const Quat& Imu::get_ang_pos()
{
	return ang_pos;
}

/**
 * @brief Returns angular velocity [rad/s]
 */
const Vector<3>& Imu::get_ang_vel()
{
	return ang_vel;
}

/**
 * @brief Returns local linear acceleration [m/s^2]
 */
const Vector<3>& Imu::get_lin_acc()
{
	return lin_acc;
}

/**
 * @brief Returns true if UAV is flipped upside down
 */
bool Imu::is_flipped()
{
	return (sqa(ang_pos.x) + sqa(ang_pos.y)) > 0.5f;
}
