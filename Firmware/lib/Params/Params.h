/**
 * @file Params.h
 * @brief Class for UAV flight parameters
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once

class Params
{
public:
	Params();		// Default constructor
	float thr_min;	// Min linear throttle [0, 1]
	float thr_max;	// Max linear throttle [0, 1]
	float qx_kp;	// Quat-x P-gain [thr/rad]
	float qx_ki;	// Quat-x I-gain [thr/(rad*s)]
	float qx_kd;	// Quat-x D-gain [thr/(rad/s)]
	float qx_ff;	// Quat-x feed-forward [thr]
	float qy_kp;	// Quat-y P-gain [thr/rad]
	float qy_ki;	// Quat-y I-gain [thr/(rad*s)]
	float qy_kd;	// Quat-y D-gain [thr/(rad/s)]
	float qy_ff;	// Quat-y feed-forward [thr]
	float qz_kp;	// Quat-z P-gain [thr/rad]
	float qz_ki;	// Quat-z I-gain [thr/(rad*s)]
	float qz_kd;	// Quat-z D-gain [thr/(rad/s)]
	float qz_ff;	// Quat-z feed-forward [thr]
};
