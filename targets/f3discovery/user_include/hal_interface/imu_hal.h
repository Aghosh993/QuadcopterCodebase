#ifndef IMU_HAL_H_
#define IMU_HAL_H_	1

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

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

/*
	Units: accelerations -> m*s^-2
			angular rates -> radians*s^-1
			magnetic field strengths -> mT
 */

typedef struct imu_descriptor{
	uint8_t _has_accelerometer;
	uint8_t _has_gyro;
	uint8_t _has_magnetometer;
	uint8_t _has_temp_sensor;

	void (*read_reg)(uint8_t reg, uint8_t *buf);
	void (*write_reg)(uint8_t reg, uint8_t val);

	void (*init)(void);
	int (*setAccScale)(struct imu_descriptor *fd, ACC_SCALE as);
	int (*setGyroScale)(struct imu_descriptor *fd, GYRO_SCALE gs);
	int (*setMagScale)(struct imu_descriptor *fd, MAG_SCALE ms);
	int (*read_raw_accel)(struct imu_descriptor *fd);
	int (*read_raw_gyro)(struct imu_descriptor *fd);
	int (*read_raw_magnetometer)(struct imu_descriptor *fd);
	int (*read_raw_temp_sensor)(struct imu_descriptor *fd);
	int (*readAllData)(struct imu_descriptor *fd);

	float acc_scale[3];
	float gyro_scale[3];
	float mag_scale[3];

	float acc_offset[3];
	float gyro_offset[3];
	float mag_offset[3];

	float acc_sign[3];
	float gyro_sign[3];
	float mag_sign[3];

	int16_t accRegisterDump[3];
	int16_t gyroRegisterDump[3];
	int16_t magRegisterDump[3];
	int16_t tempRegisterDump;
} imu_desc;

/*
	Convention: All arrays in order X-Y-Z in body frame:
 */
typedef struct {
	float accel_data[3];
	float gyro_data[3];
	float magnetometer_data[3];
	float temp_sensor_deg_c;
} imu_scaled_data_struct;

int imu_hal_init(imu_desc *imu, ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m);
int imu_readScaled(imu_desc *imu, imu_scaled_data_struct *dst);

#endif