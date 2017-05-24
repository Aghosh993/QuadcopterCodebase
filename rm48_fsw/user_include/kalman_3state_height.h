#ifndef KALMAN_3STATE_HEIGHT_H_
#define KALMAN_3STATE_HEIGHT_H_	1

/*
	BSD 3-Clause License

	Copyright (c) 2017, Abhimanyu Ghosh
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright notice, this
	list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	* Neither the name of the copyright holder nor the names of its
	contributors may be used to endorse or promote products derived from
	this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	This file contains interface functions necessary to invoke a 4-state (linear) Kalman Filter that estimates Roll, Pitch and the respective rates.
	The filter has not yet been vibration-tested but preliminary manual shake tests indicate the data should be pretty good. Obviously, the Q and R matrices 
	must be initialized correctly for your particular system and sensor combo.
 */

#include "kalman_3state_height_autogen.h"
#include "gnc_common.h"
#include "matrix.h"

#if !defined SITL_TARGET
	#include "imu.h"
#else
	#include "imu_test_harness.h"
#endif

typedef struct {
	state_estimate state_vector;
	observation sensor_data;
	imu_scaled_data_struct *imu_data;

	mat G;
	mat P;
	mat Q;
	mat R;

	float filter_dt_seconds;
} kalman_3state_struct;

int kalman_3state_init(kalman_3state_struct *s, imu_scaled_data_struct *imu_str, float dt_sec);
void kalman_3state_update(kalman_3state_struct *s, float sonar_dist);
void kalman_3state_accel_update(kalman_3state_struct *s, float accel_dt);
void kalman_3state_lidar_update(kalman_3state_struct *s, float sonar_dist, float sonar_dt);

#endif