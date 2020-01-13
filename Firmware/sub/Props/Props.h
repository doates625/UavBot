/**
 * @file Props.h
 * @brief Subsystem for UAV propeller motor controllers
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <LinearCpp.h>

/**
 * Namespace Declaration
 */
namespace Props
{
	void init();
	void set_thr(const Vector<4>& thr);
	const Vector<4>& get_thr();
}
