#ifndef IMU_H
#define IMU_H	1

#include <math.h>

#include <imu_HAL.h>

#define AXIS_X	1
#define AXIS_Y	0
#define AXIS_Z	2

#define AXIS_ROLL	AXIS_Y
#define AXIS_PITCH	AXIS_X
#define AXIS_YAW	AXIS_Z

#define ACC_X_SIGN	1.0f
#define ACC_Y_SIGN	1.0f
#define ACC_Z_SIGN	-1.0f

#define GYRO_X_SIGN	-1.0f
#define GYRO_Y_SIGN	-1.0f
#define GYRO_Z_SIGN	1.0f

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