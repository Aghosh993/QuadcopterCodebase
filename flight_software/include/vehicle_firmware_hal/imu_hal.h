#ifndef IMU_HAL_H
#define IMU_HAL_H	1

#include <hal_common_includes.h>

#define I2C2_GPIO		GPIOA

#define I2C2_SCL_PIN	GPIO9
#define I2C2_SDA_PIN	GPIO10

/*
	Direction conventions:

	The USB ports on the STM32F3Discovery board shall face towards the positive Y axis of the vehicle,
	towards the side with motors 1 and 2

	P2 shall face towards the positive X axis of the vehicle, towards the side with motors 2 and 3

	The Z axis shall be positive, pointing DOWNWARDS
 */

#define AXIS_X	0
#define AXIS_Y	1
#define AXIS_Z	2

#define AXIS_ROLL	AXIS_Y
#define AXIS_PITCH	AXIS_X
#define AXIS_YAW	AXIS_Z

/*
	LSM303DLHC I2C addresses for accelerometer and magnetometer devices:
 */
#define ADDR_LSM303DLHC_ACC	0x19

#define LSM303_CTRL_REG1_A	0x20

#define LSM303_ACC_XEN	(1<<0)
#define LSM303_ACC_YEN	(1<<1)
#define LSM303_ACC_ZEN	(1<<2)

#define LSM303_ACC_ODR_1344HZ_MASK	(1<<7) | (1<<4)

#define LSM303_CTRL_REG4_A	0x23

#define LSM303_ACC_HIGHRES_MODE_MASK	(1<<3)
#define LSM303_ACC_2G_MASK				(0<<4) | (0<<5)
#define LSM303_ACC_4G_MASK				(1<<4) | (0<<5)
#define LSM303_ACC_8G_MASK				(0<<4) | (1<<5)
#define LSM303_ACC_16G_MASK				(1<<4) | (1<<5)
#define LSM303_ACC_BDU					(1<<7)

#define LSM303_ACC_OUT_X_L	0x28
#define LSM303_ACC_OUT_X_H	0x29

#define LSM303_ACC_OUT_Y_L	0x2A
#define LSM303_ACC_OUT_Y_H	0x2B

#define LSM303_ACC_OUT_Z_L	0x2C
#define LSM303_ACC_OUT_Z_H	0x2D

#define ADDR_LSM303DLHC_MAG	0x1E

#define LSM303_CRA_REG_M	0x00

#define LSM303_MAG_DO_75HZ_MASK			(0<<2) | (1<<3) | (1<<4)
#define LSM303_MAG_DO_220HZ_MASK		(1<<2) | (1<<3) | (1<<4)

#define LSM303_CRB_REG_M	0x01

#define LSM303_MAG_1POINT3_GAUSS_MASK	(1<<5) | (0<<6) | (0<<7)
#define LSM303_MAG_1POINT9_GAUSS_MASK	(0<<5) | (1<<6) | (0<<7)
#define LSM303_MAG_2POINT5_GAUSS_MASK	(1<<5) | (1<<6) | (0<<7)
#define LSM303_MAG_4POINT0_GAUSS_MASK	(0<<5) | (0<<6) | (1<<7)
#define LSM303_MAG_4POINT7_GAUSS_MASK	(1<<5) | (0<<6) | (1<<7)
#define LSM303_MAG_5POINT6_GAUSS_MASK	(0<<5) | (1<<6) | (1<<7)
#define LSM303_MAG_8POINT1_GAUSS_MASK	(1<<5) | (1<<6) | (1<<7)

#define LSM303_MR_REG_M		0x02

#define LSM303_MAG_CONTINUOUS_CONV_MASK	(0<<0) | (0<<1)

#define LSM303_MAG_OUT_X_H	0x03
#define LSM303_MAG_OUT_X_L	0x04

#define LSM303_MAG_OUT_Z_H	0x05
#define LSM303_MAG_OUT_Z_L	0x06

#define LSM303_MAG_OUT_Y_H	0x07
#define LSM303_MAG_OUT_Y_L	0x08

#define L3GD20_CS_GPIO 			GPIOE, GPIO3

#define L3GD20_GYRO_CTRL_REG1	0x20

#define L3GD20_GYRO_XEN			(1<<1)
#define L3GD20_GYRO_YEN			(1<<0)
#define L3GD20_GYRO_ZEN			(1<<2)

#define LSGD20_PD_NORMAL_MODE	(1<<3)

#define L3GD20_GYRO_CTRL_REG4	0x23

#define L3GD20_GYRO_250DPS_MASK		(0<<4) | (0<<5)
#define L3GD20_GYRO_500DPS_MASK		(1<<4) | (0<<5)
#define L3GD20_GYRO_2000DPS_MASK	(1<<4) | (1<<5)

#define L3GD20_GYRO_SEQUENTIAL_READ_MASK		(1<<6)
#define L3GD20_GYRO_READ_MASK					(1<<7)

#define L3GD20_GYRO_OUT_X_L			0x28
#define L3GD20_GYRO_OUT_X_H			0x29

#define L3GD20_GYRO_OUT_Y_L			0x2A
#define L3GD20_GYRO_OUT_Y_H			0x2B

#define L3GD20_GYRO_OUT_Z_L			0x2C
#define L3GD20_GYRO_OUT_Z_H			0x2D
/*
	Maximum raw values readable from device data registers, per ST datasheets:
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
} imu_scaled_data_struct;

typedef struct {
	int16_t accel_data[3];
	int16_t gyro_data[3];
	int16_t magnetometer_data[3];
} imu_raw_data_struct;

void imu_i2c_bus_clear(void);

void initialize_imu(ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, imu_scaled_data_struct* buf);

void get_raw_imu_data(imu_raw_data_struct* buffer);

#endif