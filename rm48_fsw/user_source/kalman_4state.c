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

#include "kalman_4state.h"

static float scale_vector(float *data, float newmag, int n)
{
	float mag = 0.0f;

	int i = 0;
	for(i=0; i<n; ++i)
	{
		mag += data[i] * data[i];
	}

	mag = sqrtf(mag);

	float scale_factor = newmag/mag;
	
	if(mag != 0.0f)
	{
		for(i=0; i<n; ++i)
		{
			data[i] *= scale_factor;
		}
	}
}

int kalman_4state_init(kalman_4state_struct *s, ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, float dt_sec)
{
	s->state_vector.roll = 0.0f;
	s->state_vector.pitch = 0.0f;
	s->state_vector.yaw = 0.0f;

	s->state_vector.roll_rate = 0.0f;
	s->state_vector.pitch_rate = 0.0f;
	s->state_vector.yaw_rate = 0.0f;

	s->state_vector.x = 0.0f;
	s->state_vector.y = 0.0f;
	s->state_vector.z = 0.0f;

	s->state_vector.v_x = 0.0f;
	s->state_vector.v_y = 0.0f;
	s->state_vector.v_z = 0.0f;

	s->state_vector.a_x = 0.0f;
	s->state_vector.a_y = 0.0f;
	s->state_vector.a_z = 0.0f;

	s->sensor_data.gps_lat = 0.0f;
	s->sensor_data.gps_long = 0.0f;
	s->sensor_data.gps_speed = 0.0f;
	s->sensor_data.gps_course = 0.0f;

	s->sensor_data.optical_flow_v_x = 0.0f;
	s->sensor_data.optical_flow_v_y = 0.0f;

	s->sensor_data.sonar_height = 0.0f;
	s->sensor_data.lidar_height = 0.0f;
	s->sensor_data.pressure_pa = 0.0f;

	s->filter_dt_seconds = dt_sec;

	matrix_init(&(s->G), 4, 4);
	float G_init[16] = {0.6f, 0.0f, 0.0f, 0.0f,
						0.0f, 0.6f, 0.0f, 0.0f,
						0.0f, 0.0f, 0.6f, 0.0f,
						0.0f, 0.0f, 0.0f, 0.6f};
	matpopulate(&(s->G), G_init);

	matrix_init(&(s->P), 4, 4);
	float P_init[16] = {0.1f, 0.0f, 0.0f, 0.0f,
						0.0f, 0.1f, 0.0f, 0.0f,
						0.0f, 0.0f, 0.1f, 0.0f,
						0.0f, 0.0f, 0.0f, 0.1f};
	matpopulate(&(s->P), P_init);

	matrix_init(&(s->Q), 4, 4);
	float Q_init[16] = {0.001f, 0.0f, 0.0f, 0.0f,
						0.0f, 0.001f, 0.0f, 0.0f,
						0.0f, 0.0f, 0.1f, 0.0f,
						0.0f, 0.0f, 0.0f, 0.1f};
	matpopulate(&(s->Q), Q_init);

	matrix_init(&(s->R), 4, 4);
	float R_init[16] = {100.1f, 0.0f, 0.0f, 0.0f,
						0.0f, 100.1f, 0.0f, 0.0f,
						0.0f, 0.0f, 1.1f, 0.0f,
						0.0f, 0.0f, 0.0f, 1.1f};
	matpopulate(&(s->R), R_init);

	imu_hal_init(); // Initialize IMU I2C bus
	initialize_imu(a, g, m, &(s->imu_data));
	return 0;
}

void kalman_4state_update(kalman_4state_struct *s)
{
	get_scaled_imu_data(&(s->imu_data));
	scale_vector(&(s->imu_data.accel_data[0]), 9.810f, 3);

	float state_predicted[4];
	predict_state(s->filter_dt_seconds, s->state_vector.pitch, s->state_vector.pitch_rate, s->state_vector.roll, s->state_vector.roll_rate, state_predicted);
	float covariance_predicted[16];
	predict_cov(s->filter_dt_seconds, get_value(s->P, 1, 1), get_value(s->P, 1, 2), get_value(s->P, 1, 3), get_value(s->P, 1, 4),
												get_value(s->P, 2, 1), get_value(s->P, 2, 2), get_value(s->P, 2, 3), get_value(s->P, 2, 4),
												get_value(s->P, 3, 1), get_value(s->P, 3, 2), get_value(s->P, 3, 3), get_value(s->P, 3, 4),
												get_value(s->P, 4, 1), get_value(s->P, 4, 2), get_value(s->P, 4, 3), get_value(s->P, 4, 4),
												get_value(s->Q, 1, 1), get_value(s->Q, 2, 2), get_value(s->Q, 3, 3), get_value(s->Q, 4, 4), covariance_predicted);
	s->state_vector.roll = state_predicted[0];
	s->state_vector.pitch = state_predicted[1];
	s->state_vector.roll_rate = state_predicted[2];
	s->state_vector.pitch_rate = state_predicted[3];

	matpopulate(&(s->P), covariance_predicted);

	float gain_updated[16];
	update_gain(9.810f, get_value(s->P, 1, 1), get_value(s->P, 1, 2), get_value(s->P, 1, 3), get_value(s->P, 1, 4),
						get_value(s->P, 2, 1), get_value(s->P, 2, 2), get_value(s->P, 2, 3), get_value(s->P, 2, 4),
						get_value(s->P, 3, 1), get_value(s->P, 3, 2), get_value(s->P, 3, 3), get_value(s->P, 3, 4),
						get_value(s->P, 4, 1), get_value(s->P, 4, 2), get_value(s->P, 4, 3), get_value(s->P, 4, 4),
						get_value(s->R, 1, 1), get_value(s->R, 2, 2), get_value(s->R, 3, 3), get_value(s->R, 4, 4), gain_updated);
	float state_updated[4];
	update_state(s->imu_data.accel_data[0], s->imu_data.accel_data[1], 9.810f, 
					get_value(s->G, 1, 1), get_value(s->G, 1, 2), get_value(s->G, 1, 3), get_value(s->G, 1, 4),
					get_value(s->G, 2, 1), get_value(s->G, 2, 2), get_value(s->G, 2, 3), get_value(s->G, 2, 4),
					get_value(s->G, 3, 1), get_value(s->G, 3, 2), get_value(s->G, 3, 3), get_value(s->G, 3, 4),
					get_value(s->G, 4, 1), get_value(s->G, 4, 2), get_value(s->G, 4, 3), get_value(s->G, 4, 4),
					degrees_to_radians(s->imu_data.gyro_data[1]), degrees_to_radians(s->imu_data.gyro_data[0]), 
					s->state_vector.pitch, s->state_vector.pitch_rate, s->state_vector.roll, s->state_vector.roll_rate, state_updated);
	float covariance_updated[16];
	update_cov(9.810f, get_value(s->G, 1, 1), get_value(s->G, 1, 2), get_value(s->G, 1, 3), get_value(s->G, 1, 4),
						get_value(s->G, 2, 1), get_value(s->G, 2, 2), get_value(s->G, 2, 3), get_value(s->G, 2, 4),
						get_value(s->G, 3, 1), get_value(s->G, 3, 2), get_value(s->G, 3, 3), get_value(s->G, 3, 4),
						get_value(s->G, 4, 1), get_value(s->G, 4, 2), get_value(s->G, 4, 3), get_value(s->G, 4, 4),
						get_value(s->P, 1, 1), get_value(s->P, 1, 2), get_value(s->P, 1, 3), get_value(s->P, 1, 4),
						get_value(s->P, 2, 1), get_value(s->P, 2, 2), get_value(s->P, 2, 3), get_value(s->P, 2, 4),
						get_value(s->P, 3, 1), get_value(s->P, 3, 2), get_value(s->P, 3, 3), get_value(s->P, 3, 4),
						get_value(s->P, 4, 1), get_value(s->P, 4, 2), get_value(s->P, 4, 3), get_value(s->P, 4, 4), covariance_updated);

	matpopulate(&(s->G), gain_updated);
	s->state_vector.roll = state_updated[0];
	s->state_vector.pitch = state_updated[1];
	s->state_vector.roll_rate = state_updated[2];
	s->state_vector.pitch_rate = state_updated[3];
	matpopulate(&(s->P), covariance_updated);
}