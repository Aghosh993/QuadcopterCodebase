#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_	1

#include <stdint.h>
#include <math.h>

#if defined USE_STDIO_DEBUG
	#include <stdio.h>
#endif

#if !defined SITL_TARGET
	#include "imu_hal.h"
#else
	#include "imu_test_harness.h"
#endif
#include "gnc_common.h"

#define FIRSTORDER_ACCEL_WEIGHT		0.02f
#define FIRSTORDER_GYRO_WEIGHT		0.98f

typedef enum {
	MODE_1STORDER_COMPFILTER,
	MODE_2NDORDER_COMPFILTER
} filter_mode;

typedef struct {
	state_estimate state_vector;
	observation sensor_data;
	imu_scaled_data_struct *imu_data;

	float roll_integral;
	float pitch_integral;

	float k_P;
	float k_I;
	float filter_dt_seconds;

	filter_mode m;
} complementary_filter_struct;

int init_complementary_filter(complementary_filter_struct *s, imu_scaled_data_struct *imu_str, 
								float dt_sec, float omega_natural, float damping_ratio, filter_mode fm);
void update_complementary_filter(complementary_filter_struct *s);

#if defined USE_STDIO_DEBUG
	void test_imu_acquisition(complementary_filter_struct *s)
#endif

#endif