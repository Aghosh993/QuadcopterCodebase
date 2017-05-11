#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_	1

#include <stdint.h>
#include <math.h>

#if defined USE_STDIO_DEBUG
	#include <stdio.h>
#endif

#if !defined SITL_TARGET
	#include "imu.h"
#else
	#include "imu_test_harness.h"
#endif

/*
	Conventions: 
		+x: Forward	
		+y: Right
		+z: Down

		Roll to the RIGHT, Pitch UP (Nose up) and Turning rightware induce a positive change in Roll, Pitch and Yaw respectively
 */

typedef struct {
	float roll;
	float pitch;
	float yaw;

	float roll_rate;
	float pitch_rate;
	float yaw_rate;

	float x;
	float y;
	float z;

	float v_x;
	float v_y;
	float v_z;

	float a_x;
	float a_y;
	float a_z;	
} state_estimate;

typedef struct {
	double gps_lat;
	double gps_long;
	float gps_speed;
	float gps_course;

	float optical_flow_v_x;
	float optical_flow_v_y;

	float sonar_height;
	float lidar_height;
	float pressure_pa;
} observation;

typedef struct {
	state_estimate state_vector;
	observation sensor_data;
	imu_scaled_data_struct imu_data;

	float roll_integral;
	float pitch_integral;

	float k_P;
	float k_I;
	float filter_dt_seconds;
} complementary_filter_struct;

int init_complementary_filter(complementary_filter_struct *s, ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, 
								float dt_sec, float omega_natural, float damping_ratio);
void update_complementary_filter(complementary_filter_struct *s);
float degrees_to_radians(float deg);
float radians_to_degrees(float rad);

#if defined USE_STDIO_DEBUG
	void test_imu_acquisition(complementary_filter_struct *s)
#endif

#endif