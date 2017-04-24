#ifndef IMU_HAL_H
#define IMU_HAL_H	1

#include <hal_common_includes.h>

/*
	Direction conventions:

	The 50-pin edge connector on the Autopilot shall face towards the positive Y axis of the vehicle,
	towards the side with motors 1 and 2

	The JTAG conenctor shall be on the TOP, and shall face towards the positive X axis of the vehicle, towards the side with motors 2 and 3

	The Z axis shall be positive, pointing DOWNWARDS
 */

#define TIMEOUT_COUNTER_MAX		200000

/*
	MPU-9250 device I2C address:
 */

#define MPU9250_I2C_ADDR	0x68

/*
	MPU-9250 device configuration register addresses:
 */

#define MPU9250_WHO_AM_I_REG		0x75
#define MPU9250_CONFIG				0x1A
#define MPU9250_GYRO_CONFIG			0x1B
#define MPU9250_ACCEL_CONFIG		0x1C
#define MPU9250_ACCEL_CONFIG2		0x1D
	// #define ACCEL_LPF_21_2_HZ			1
	// #define ACCEL_LPF_99_HZ				1
	#define ACCEL_LPF_218_1_HZ				1

/*
	MPU-9250 device data register addresses:
 */
#define MPU9250_ACCEL_XOUT_H		0x3B
#define MPU9250_ACCEL_XOUT_L		0x3C
#define MPU9250_ACCEL_YOUT_H		0x3D
#define MPU9250_ACCEL_YOUT_L		0x3E
#define MPU9250_ACCEL_ZOUT_H		0x3F
#define MPU9250_ACCEL_ZOUT_L		0x40

#define MPU9250_GYRO_XOUT_H			0x43
#define MPU9250_GYRO_XOUT_L			0x44
#define MPU9250_GYRO_YOUT_H			0x45
#define MPU9250_GYRO_YOUT_L			0x46
#define MPU9250_GYRO_ZOUT_H			0x47
#define MPU9250_GYRO_ZOUT_L			0x48

#define MPU9250_TEMP_OUT_H			0x41
#define MPU9250_TEMP_OUT_L			0x42

/*
	Maximum raw values readable from device data registers, per datasheets:
 */
#define ACC_MAX_RAW_POS_OUTPUT		32767
#define ACC_MAX_RAW_NEG_OUTPUT		-32768

#define GYRO_MAX_RAW_POS_OUTPUT		32767
#define GYRO_MAX_RAW_NEG_OUTPUT		-32768

#define MAG_MAX_RAW_POS_OUTPUT		2047
#define MAG_MAX_RAW_NEG_OUTPUT		-2048

typedef enum {
	SCALE_2G,
	SCALE_4G,
	SCALE_8G,
	SCALE_16G
} ACC_SCALE;

typedef enum {
	SCALE_250_DPS,
	SCALE_500_DPS,
	SCALE_1000_DPS,
	SCALE_2000_DPS
} GYRO_SCALE;

typedef enum {
	SCALE_1POINT3_GAUSS,
	SCALE_1POINT9_GAUSS,
	SCALE_2POINT5_GAUSS,
	SCALE_4POINT0_GAUSS,
	SCALE_4POINT7_GAUSS,
	SCALE_5POINT6_GAUSS,
	SCALE_8POINT1_GAUSS
} MAG_SCALE;

typedef struct {
	ACC_SCALE acc_meas_scale;
	float accel_data[3];

	GYRO_SCALE gyro_meas_scale;
	float gyro_data[3];

	MAG_SCALE mag_meas_scale;
	float magnetometer_data[3];

	float temp_sensor_deg_c;
} imu_scaled_data_struct;

typedef struct {
	int16_t accel_data[3];
	int16_t gyro_data[3];
	int16_t magnetometer_data[3];
	int16_t temp_sensor_data;
} imu_raw_data_struct;

void imu_hal_init(void);

void initialize_imu(ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, imu_scaled_data_struct* buf);

int get_raw_imu_data(imu_raw_data_struct* buffer);

#endif