/*
 * File: comp_filter.h
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2016
 */

#ifndef COMP_FILTER_
#define COMP_FILTER_	1

#include "iir_filters.h"
#include "imu.h"
#include "rt_telemetry.h"

// #define DEBUG_BIAS_VALUES	1

/*
	Standard includes:
 */
#include <math.h>

#define ACCEL_WEIGHT 	0.02f //2%
#define GYRO_WEIGHT		0.98f //98%

// #define ACC_USE_SW_LPF	1

typedef struct
{
	float roll;
	float pitch;
	float yaw;

	float vertical_dynamic_acceleration_post_lpf;
	float vertical_velocity;
	float height;

	float x_acceleration;
	float y_acceleration;

	float x_velocity;
	float y_velocity;

	float x_displacement;
	float y_displacement;
} filtered_quadrotor_state;

typedef struct
{
	float roll_gyro_bias;
	float pitch_gyro_bias;
	float yaw_gyro_bias;

	float x_accelerometer_bias;
	float y_accelerometer_bias;
	float z_accelerometer_bias;
} vehicle_bias_data_struct;

#define GYRO_X_IDEAL_READING 	0.0f	// 0 deg/s
#define GYRO_Y_IDEAL_READING 	0.0f	// 0 deg/s
#define GYRO_Z_IDEAL_READING 	0.0f	// 0 deg/s

#define ACCELEROMETER_X_IDEAL_READING 0.0f		// 0 g
#define ACCELEROMETER_Y_IDEAL_READING 0.0f		// 0 g
#define ACCELEROMETER_Z_IDEAL_READING 9.810f 	// 1 g

// LPF (Lowpass Filter) parameters:

#define ACCEL_X_LPF_CUTOFF_HERTZ 	80.0f //100.0f
#define ACCEL_Y_LPF_CUTOFF_HERTZ 	80.0f //100.0f
#define ACCEL_Z_LPF_CUTOFF_HERTZ 	80.0f //100.0f

// Bias calculation number of samples to take:

#define BIAS_CALC_NUM_SAMPLES	500

// Bias calculation milliseconds between samples:

#define BIAS_CALC_SAMPLE_DT_MS	20

/*
 * HPF (Highpass Filter) and LPF (Low Pass Filter) parameters:
 *
 * Should eliminate relatively low-frequency walk(drift) in gyro rate data,
 * while also reducing vibration propagation from vehicle frame to IMU via PCB.
 */

#define GYRO_HPF_CUTOFF_HERTZ 		5.0f
#define GYRO_LPF_CUTOFF_HERTZ		10.0f

#define ACCEL_Z_HPF_CUTOFF_HERTZ	1.5f

/*
 * Only applies HPF (Highpass filter) to gyro data:
 */
// #define GYRO_HPF_ENABLED_X	1
// #define GYRO_HPF_ENABLED_Y	1

#define MAGNETOMETER_LPF_CUTOFF		100.0f

/*
	Function prototypes:
 */

void init_comp_filter(filtered_quadrotor_state* stvar);
void get_corrected_scaled_gyro_data(imu_scaled_data_struct* input, float* output);
void get_corrected_scaled_accelerometer_data(imu_scaled_data_struct* input, float* output);
void get_filtered_vehicle_state(filtered_quadrotor_state* statevar, imu_scaled_data_struct* input);
void do_bias_calculation(imu_scaled_data_struct *imu_data);

#endif
