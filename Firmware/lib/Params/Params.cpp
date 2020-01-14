/**
 * @file Params.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Params.h"

/**
 * @brief Default constructor
 */
Params::Params() :
	thr_min(0.0f),
	thr_max(0.0f),
	qx_kp(0.0f),
	qx_ki(0.0f),
	qx_kd(0.0f),
	qx_ff(0.0f),
	qy_kp(0.0f),
	qy_ki(0.0f),
	qy_kd(0.0f),
	qy_ff(0.0f),
	qz_kp(0.0f),
	qz_ki(0.0f),
	qz_kd(0.0f),
	qz_ff(0.0f)
{
	return;
}
