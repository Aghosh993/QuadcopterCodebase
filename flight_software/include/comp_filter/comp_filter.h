/*
 * File: comp_filter.h
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2015
 */

#ifndef COMP_FILTER_H_
#define COMP_FILTER_H_

#include <math.h>
#include <imu_hal.h> // For IMU data structure definitions

#include <imu.h>
#include "mission_timekeeper.h"

#define DEBUG_BIAS_VALUES 1

#ifdef DEBUG_BIAS_VALUES
 #include <stdio.h>
#endif

#define PI_VAL			3.14159f

#define RADIANS_TO_DEGREES_CONVERSION_FACTOR	((float)180.0/(float)PI_VAL)
#define DEGREES_TO_RADIANS_CONVERSION_FACTOR	((float)PI_VAL/(float)180.0)

// Accelerometer weight: 2%, Gyro weight: 98%:

#define ACCEL_WEIGHT 	0.02f //2%
#define GYRO_WEIGHT		0.98f //98%
// #define ACCEL_WEIGHT 	0.04f //4%
// #define GYRO_WEIGHT		0.96f //96%

#define DT_FILTER_LOOP	0.001f // Interval at which the filter is run, in seconds

// LPF (Lowpass Filter) parameters:

#define ACCEL_X_LPF_CUTOFF_HERTZ 	100.0f //10.0f
#define ACCEL_Y_LPF_CUTOFF_HERTZ 	100.0f //10.0f
#define ACCEL_Z_LPF_CUTOFF_HERTZ 	100.0f

// Bias calculation number of samples to take:

#define BIAS_CALC_NUM_SAMPLES	500

// Bias calculation milliseconds between samples:

#define BIAS_CALC_SAMPLE_DT_MS	20

// LPF-related data structure(s):

typedef struct
{
	float prev_output;
	float alpha_val;
} lpf_data_struct;

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

// LPF function prototypes:
void init_lpf_variables(lpf_data_struct* lpf_str, float cutoff_freq_hertz);
float lowpass_filter(float input, lpf_data_struct* lpf_str);

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
 * Applies both HPF and LPF (Highpass filter and Lowpass filter)
 * to gyro data:
 */

//#define GYRO_HPF_LPF_ENABLED_X	1
//#define GYRO_HPF_LPF_ENABLED_Y	1

// LPF-related data structure(s):

typedef struct
{
	float prev_input;
	float prev_output;
	float alpha_val;
} hpf_data_struct;

// HPF function prototypes:
void init_hpf_variables(hpf_data_struct* hpf_str, float cutoff_freq_hertz);
float highpass_filter(float input, hpf_data_struct* hpf_str);

typedef struct
{
	float roll;
	float pitch;
	float yaw;

	float vertical_dynamic_acceleration_post_lpf;
	float vertical_velocity;
	float height;

	float x_velocity;
	float y_velocity;

	float x_displacement;
	float y_displacement;
} filtered_quadrotor_state;

void init_comp_filter(filtered_quadrotor_state* stvar);
void get_corrected_scaled_gyro_data(imu_scaled_data_struct* input, float* output);
void get_corrected_scaled_accelerometer_data(imu_scaled_data_struct* input, float* output);
void get_filtered_vehicle_state(filtered_quadrotor_state* statevar, imu_scaled_data_struct* input);
// void propagate_compensated_vehicle_height(float height_vehicle_body_frame, filtered_quadrotor_state *st);
void propagate_compensated_vehicle_height(float height_vehicle_body_frame, 
											float height_sensor_position_body_frame_x, 
											float height_sensor_position_body_frame_y, 
											filtered_quadrotor_state *st);

void do_bias_calculation(imu_scaled_data_struct *imu_data);

#endif /* COMP_FILTER_H_ */
