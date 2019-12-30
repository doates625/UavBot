/**
 * @file Bluetooth.h
 * @brief Subsystem for Bluetooth remote control
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <LinearCpp.h>

/**
 * Namespace Declaration
 */
namespace Bluetooth
{
	void init();
	bool update();
	const Vector<3>& get_acc_cmd();
	float get_heading_cmd();
}
