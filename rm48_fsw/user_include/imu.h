#ifndef IMU_H
#define IMU_H	1

#include <stdint.h>
#include <math.h>

#include <imu_hal.h>

#define ACC_SCALE_2G				9.810f*0.000061035f
#define ACC_SCALE_4G				9.810f*0.00012207f
#define ACC_SCALE_8G				9.810f*0.000244141f
#define ACC_SCALE_16G				9.810f*0.000488281f

#define GYRO_SCALE_250_DPS			0.007629395f
#define GYRO_SCALE_500_DPS			0.015258789f
#define GYRO_SCALE_1000_DPS			0.030517578f
#define GYRO_SCALE_2000_DPS			0.061035156f

#define TEMP_SCALE					1.0f 	// Need to verify this!!
#define TEMP_OFFSET					5900U  	// Need to verify this!!

int get_scaled_imu_data(imu_scaled_data_struct* buffer);

#endif