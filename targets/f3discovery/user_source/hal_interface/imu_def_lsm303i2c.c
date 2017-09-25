#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "hal_common_includes.h"
#include "imu_hal.h"

// LSM303 defines and methods:

#define LSM303_ACCEL_AXIS_X	0
#define LSM303_ACCEL_AXIS_Y	1
#define LSM303_ACCEL_AXIS_Z	2

#define LSM303_MAG_AXIS_X	0
#define LSM303_MAG_AXIS_Y	1
#define LSM303_MAG_AXIS_Z	2

#define LSM303_ACC_X_SIGN	-1.0f
#define LSM303_ACC_Y_SIGN	1.0f
#define LSM303_ACC_Z_SIGN	1.0f

#define LSM303_GYRO_ROLL_SIGN	1.0f
#define LSM303_GYRO_PITCH_SIGN	1.0f
#define LSM303_GYRO_YAW_SIGN	1.0f

#define LSM303_MAG_X_SIGN	1.0f
#define LSM303_MAG_Y_SIGN	-1.0f
#define LSM303_MAG_Z_SIGN	-1.0f

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

static volatile I2C_HandleTypeDef imu_i2c_bus;

typedef enum {
	BAUD_100KHZ,
	BAUD_400KHZ
} i2cbaud;

static void i2c_setup(i2cbaud b)
{
	imu_i2c_bus.Instance = 				I2C1;
	imu_i2c_bus.Init.OwnAddress1 = 		0x32;
	imu_i2c_bus.Init.AddressingMode = 	I2C_ADDRESSINGMODE_7BIT;
	imu_i2c_bus.Init.GeneralCallMode = 	I2C_GENERALCALL_DISABLE;
	imu_i2c_bus.Init.NoStretchMode = 	I2C_NOSTRETCH_DISABLE;
	imu_i2c_bus.Mode = 					HAL_I2C_MODE_MASTER;

	switch(b)
	{
		case BAUD_100KHZ:
			imu_i2c_bus.Init.Timing = 0x10321111; // Experimentally-verified SCL of around 97 kHz at System CLK of 72 MHz
			break;
		case BAUD_400KHZ:
			imu_i2c_bus.Init.Timing = 0x00320202; // Experimentally-verified SCL of around 398 kHz at System CLK of 72 MHz
			break;
	}

	HAL_I2C_Init(&imu_i2c_bus);
}

static volatile uint8_t lsm303_acc_reg1_mask, lsm303_acc_reg4_mask;
static volatile uint8_t lsm303_mag_cra_mask, lsm303_mag_crb_mask, lsm303_mag_mr_mask;

static void lsm303_init(void)
{
	i2c_setup(BAUD_400KHZ);
	lsm303_acc_reg1_mask = LSM303_ACC_XEN | LSM303_ACC_YEN | LSM303_ACC_ZEN |
									LSM303_ACC_ODR_1344HZ_MASK;
	lsm303_acc_reg4_mask = LSM303_ACC_HIGHRES_MODE_MASK;
	lsm303_mag_cra_mask = LSM303_MAG_DO_220HZ_MASK;
	lsm303_mag_crb_mask = 0U;
	lsm303_mag_mr_mask = LSM303_MAG_CONTINUOUS_CONV_MASK;

	/*
		Write initialization data to accelerometer:
	 */

	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_ACC << 1, LSM303_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &lsm303_acc_reg1_mask, 1U, 100U);
	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_ACC << 1, LSM303_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &lsm303_acc_reg4_mask, 1U, 100U);
	
	/*
		Write initialization data to magnetometer:
	 */
	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_MAG << 1, LSM303_CRA_REG_M, I2C_MEMADD_SIZE_8BIT, &lsm303_mag_cra_mask, 1U, 100U);
	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_MAG << 1, LSM303_CRB_REG_M, I2C_MEMADD_SIZE_8BIT, &lsm303_mag_crb_mask, 1U, 100U);
	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_MAG << 1, LSM303_MR_REG_M, I2C_MEMADD_SIZE_8BIT, &lsm303_mag_mr_mask, 1U, 100U);
}

static int lsm303_setAccScale(struct imu_descriptor *fd, ACC_SCALE as)
{
	switch(as)
	{
		case SCALE_2G:
			lsm303_acc_reg4_mask |= LSM303_ACC_2G_MASK;
			break;
		case SCALE_4G:
			lsm303_acc_reg4_mask |= LSM303_ACC_4G_MASK;
			break;
		case SCALE_8G:
			lsm303_acc_reg4_mask |= LSM303_ACC_8G_MASK;
			break;
		case SCALE_16G:
			lsm303_acc_reg4_mask |= LSM303_ACC_16G_MASK;
			break;
		default:
			return -1;
	}
	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_ACC << 1, LSM303_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &lsm303_acc_reg4_mask, 1U, 100U);
	return 0;
}

static int lsm303_setMagScale(struct imu_descriptor *fd, MAG_SCALE ms)
{
	switch(ms)
	{
		case SCALE_1POINT3_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_1POINT3_GAUSS_MASK;
			fd->mag_scale[0] = 0.00211806210598522f;
			fd->mag_scale[1] = 0.00201555099196036f;
			fd->mag_scale[2] = 0.00206958517466016f;
			fd->mag_offset[0] = -35.8068088995649f;
			fd->mag_offset[1] = 68.3436954958760f;
			fd->mag_offset[2] = 67.5442900945970f;


			break;
		case SCALE_1POINT9_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_1POINT9_GAUSS_MASK;
			break;
		case SCALE_2POINT5_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_2POINT5_GAUSS_MASK;
			break;
		case SCALE_4POINT0_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_4POINT0_GAUSS_MASK;
			break;
		case SCALE_4POINT7_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_4POINT7_GAUSS_MASK;
			break;
		case SCALE_5POINT6_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_5POINT6_GAUSS_MASK;
			break;
		case SCALE_8POINT1_GAUSS:
			lsm303_mag_crb_mask |= LSM303_MAG_8POINT1_GAUSS_MASK;
			break;
		default:
			return -1;
	}
	HAL_I2C_Mem_Write(&imu_i2c_bus, ADDR_LSM303DLHC_MAG << 1, LSM303_CRB_REG_M, I2C_MEMADD_SIZE_8BIT, &lsm303_mag_crb_mask, 1U, 100U);
	return 0;
}

static int lsm303_readAllData(struct imu_descriptor *fd)
{
	uint8_t accelerometer_buffer[6U];
	uint8_t magnetometer_buffer[6U];

	union {
		uint8_t in[2];
		int16_t output;
	} conv;

	HAL_I2C_Mem_Read(&imu_i2c_bus, ADDR_LSM303DLHC_ACC << 1, LSM303_ACC_OUT_X_L | 0x80, I2C_MEMADD_SIZE_8BIT, accelerometer_buffer, 6U, 100U);
	HAL_I2C_Mem_Read(&imu_i2c_bus, ADDR_LSM303DLHC_MAG << 1, LSM303_MAG_OUT_X_H, I2C_MEMADD_SIZE_8BIT, magnetometer_buffer, 6U, 100U);

	conv.in[0] = accelerometer_buffer[0];
	conv.in[1] = accelerometer_buffer[1];
	fd->accRegisterDump[0] = conv.output;

	conv.in[0] = accelerometer_buffer[2];
	conv.in[1] = accelerometer_buffer[3];
	fd->accRegisterDump[1] = conv.output;

	conv.in[0] = accelerometer_buffer[4];
	conv.in[1] = accelerometer_buffer[5];
	fd->accRegisterDump[2] = conv.output;

	conv.in[0] = magnetometer_buffer[1];
	conv.in[1] = magnetometer_buffer[0];
	fd->magRegisterDump[0] = conv.output;

	conv.in[0] = magnetometer_buffer[5];
	conv.in[1] = magnetometer_buffer[4];
	fd->magRegisterDump[1] = conv.output;

	conv.in[0] = magnetometer_buffer[3];
	conv.in[1] = magnetometer_buffer[2];
	fd->magRegisterDump[2] = conv.output;
}

imu_desc lsm303_i2c = {
	1U, 0U, 1U, 0U,

	NULL,
	NULL,
	
	lsm303_init,	
	lsm303_setAccScale,
	NULL,
	lsm303_setMagScale,

	NULL,
	NULL,
	NULL,
	NULL,
	lsm303_readAllData,
	
	{1.0f, 1.0f, 1.0f},
	{1.0f, 1.0f, 1.0f},
	{1.0f, 1.0f, 1.0f},

	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f},

	{LSM303_ACC_X_SIGN, LSM303_ACC_Y_SIGN, LSM303_ACC_Z_SIGN},
	{LSM303_GYRO_ROLL_SIGN, LSM303_GYRO_PITCH_SIGN, LSM303_GYRO_YAW_SIGN},
	{LSM303_MAG_X_SIGN, LSM303_MAG_Y_SIGN, LSM303_MAG_Z_SIGN},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	0
};