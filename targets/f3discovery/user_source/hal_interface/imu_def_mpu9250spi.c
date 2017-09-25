#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "hal_common_includes.h"
#include "imu_hal.h"

// MPU9250 registers and methods:

#define ACCEL_AXIS_X	0
#define ACCEL_AXIS_Y	1
#define ACCEL_AXIS_Z	2

#define AXIS_ROLL	0
#define AXIS_PITCH	1
#define AXIS_YAW	2

#define MAG_AXIS_X	0
#define MAG_AXIS_Y	1
#define MAG_AXIS_Z	2

#define MPU9250_ACC_X_SIGN	1.0f
#define MPU9250_ACC_Y_SIGN	1.0f
#define MPU9250_ACC_Z_SIGN	-1.0f

#define MPU9250_MAG_X_SIGN	1.0f
#define MPU9250_MAG_Y_SIGN	1.0f
#define MPU9250_MAG_Z_SIGN	-1.0f

#define MPU9250_GYRO_ROLL_SIGN	-1.0f
#define MPU9250_GYRO_PITCH_SIGN	-1.0f
#define MPU9250_GYRO_YAW_SIGN	1.0f

#define TEMP_SCALE					1.0f 	// Need to verify this!!
#define TEMP_OFFSET					5900U  	// Need to verify this!!

/*
	MPU-9250 device configuration register addresses:
 */

#define MPU9250_WHO_AM_I_REG		0x75
#define MPU9250_CONFIG				0x1A
#define MPU9250_GYRO_CONFIG			0x1B
#define MPU9250_ACCEL_CONFIG		0x1C
#define MPU9250_ACCEL_CONFIG2		0x1D

// #define ACCEL_NO_LPF				1
#define ACCEL_LPF_21_2_HZ			1
// #define ACCEL_LPF_99_HZ				1
// #define ACCEL_LPF_218_1_HZ			1

#define GYRO_SCALE_250_DPS			0.007629395f
#define GYRO_SCALE_500_DPS			0.015258789f
#define GYRO_SCALE_1000_DPS			0.030517578f
#define GYRO_SCALE_2000_DPS			0.061035156f

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

typedef enum {
	BAUD_1M,
	BAUD_2M,
	BAUD_5M
} spibaud;

static volatile SPI_HandleTypeDef imu_spi_bus;

static void spi_setup(spibaud b)
{
	imu_spi_bus.Instance = 				SPI1;
	imu_spi_bus.Init.Mode = 			SPI_MODE_MASTER;
	imu_spi_bus.Init.Direction = 		SPI_DIRECTION_2LINES;
	imu_spi_bus.Init.DataSize = 		SPI_DATASIZE_8BIT;
	imu_spi_bus.Init.CLKPolarity = 		SPI_POLARITY_LOW;
	imu_spi_bus.Init.CLKPhase = 		SPI_PHASE_1EDGE;
	imu_spi_bus.Init.NSS = 				SPI_NSS_SOFT;
	imu_spi_bus.Init.FirstBit = 		SPI_FIRSTBIT_MSB;
	imu_spi_bus.Init.TIMode = 			SPI_TIMODE_DISABLED;
	imu_spi_bus.Init.CRCCalculation = 	SPI_CRCCALCULATION_DISABLED;
	imu_spi_bus.Init.CRCPolynomial = 	7;

	switch(b)
	{
		case BAUD_1M:
			imu_spi_bus.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
			break;
		case BAUD_2M:
			imu_spi_bus.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
			break;
		case BAUD_5M:
			imu_spi_bus.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
			break;
	}
	HAL_SPI_Init(&imu_spi_bus);
}

static int spi_write_to_mpu9250_register(uint8_t reg, uint8_t data)
{
	/*
		Select MPU9250 for SPI transaction:
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

	/*
		Send register and data contents:
	 */

	uint8_t tx_data[2];
	tx_data[0] = reg;
	tx_data[1] = data;

	uint8_t rx_data[2];

	if(HAL_SPI_TransmitReceive(&imu_spi_bus, tx_data, rx_data, 2U, 100U) != HAL_OK)
	{
		return -1;
	}

	/*
		De-select the MPU9250 for SPI transaction:
	 */

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	return 0;
}

/*
	Function to read a sequence of bytes from l3gd20.
 */
static int spi_read_from_mpu9250_register(uint8_t start_reg, uint8_t len, uint8_t *data_buffer)
{
	uint8_t dummy_byte = 0U;
	uint8_t i = 0U;

	/*
		Select MPU9250 for SPI transaction:
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

	/*
		Send register we wish to start read sequence from, logical
		OR with masks to enable address auto-increment and read mode:
	 */
	uint8_t receive_cmd = start_reg | (1<<7); // Set MSB to indicate read operation
	
	if(HAL_SPI_TransmitReceive(&imu_spi_bus, &receive_cmd, &dummy_byte, 1U, 100U) != HAL_OK)
	{
		return -1;
	}

	/*
		Read data:
	 */
	for(i=0U; i<len; ++i)
	{
		/*
			Send zeroes and keep clocking data in until we fill the data buffer:
		 */
		if(HAL_SPI_TransmitReceive(&imu_spi_bus, (uint8_t *)&dummy_byte, (uint8_t *)&data_buffer[i], 1U, 100U) != HAL_OK)
		{
			return -3;
		}
	}

	/*
		De-select the MPU9250 for SPI transaction:
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

	return 0;
}

static void mpu9250_read_register(uint8_t reg_address, uint8_t *buf)
{
	spi_read_from_mpu9250_register(reg_address, 1U, buf);
}

static void mpu9250_write_register(uint8_t reg_address, uint8_t value)
{
	spi_write_to_mpu9250_register(reg_address, value);
}

static int32_t mpu9250_read_multiple_registers(uint8_t reg_base_address, uint8_t *buffer, uint8_t len)
{
	spi_read_from_mpu9250_register(reg_base_address, len, buffer);
	return len;
}

static volatile uint8_t accel_mask1, accel_mask2;
static volatile uint8_t gyro_mask;
static volatile uint8_t mag_mask;

static void mpu9250_init(void)
{
	#ifdef ACCEL_NO_LPF
		accel_mask2 = 0U;
	#endif
	#ifdef ACCEL_LPF_218_1_HZ
		accel_mask2 = 0 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_99_HZ
		accel_mask2 = 2 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_44_8_HZ
		accel_mask2 = 3 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_21_2_HZ
		accel_mask2 = 4 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_10_2_HZ
		accel_mask2 = 5 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_5_05_HZ
		accel_mask2 = 6 | (1<<3);
	#endif
	#ifdef ACCEL_DEC2_420_HZ
		accel_mask2 = 7 | (1<<3);
	#endif

	spi_setup(BAUD_1M);
	mpu9250_write_register(MPU9250_ACCEL_CONFIG2, accel_mask2);
}

static int mpu9250_setAccScale(struct imu_descriptor *fd, ACC_SCALE as)
{
	accel_mask1 = 0U;
	switch(as)
	{
		case SCALE_2G:
			accel_mask1 |= (0<<4) | (0<<3);
			fd->acc_scale[0] = 0.000604424670566886f;
			fd->acc_scale[1] = 0.000596504539449320f;
			fd->acc_scale[2] = 0.000587356199499023f;
			fd->acc_offset[0] = 2897.03209990163f;
			fd->acc_offset[1] = 1439.43206696994f;
			fd->acc_offset[2] = -2971.86020030435f;
			break;
		case SCALE_4G:
			accel_mask1 |= (0<<4) | (1<<3);
			// fd->acc_scale[0] = ACC_SCALE_4G;
			// fd->acc_scale[1] = ACC_SCALE_4G;
			// fd->acc_scale[2] = ACC_SCALE_4G;
			break;
		case SCALE_8G:
			accel_mask1 |= (1<<4) | (0<<3);
			// fd->acc_scale[0] = ACC_SCALE_8G;
			// fd->acc_scale[1] = ACC_SCALE_8G;
			// fd->acc_scale[2] = ACC_SCALE_8G;
			break;
		case SCALE_16G:
			accel_mask1 |= (1<<4) | (1<<3);
			// fd->acc_scale[0] = ACC_SCALE_16G;
			// fd->acc_scale[1] = ACC_SCALE_16G;
			// fd->acc_scale[2] = ACC_SCALE_16G;
			break;
	}
	mpu9250_write_register(MPU9250_ACCEL_CONFIG, accel_mask1);
	return 0;
}

static int mpu9250_setGyroScale(struct imu_descriptor *fd, GYRO_SCALE gs)
{
	gyro_mask = 0U;
	switch(gs)
	{
		case SCALE_250_DPS:
			gyro_mask |= (0<<4) | (0<<3);
			fd->gyro_scale[0] = GYRO_SCALE_250_DPS;
			fd->gyro_scale[1] = GYRO_SCALE_250_DPS;
			fd->gyro_scale[2] = GYRO_SCALE_250_DPS;
			break;
		case SCALE_500_DPS:
			gyro_mask |= (0<<4) | (1<<3);
			fd->gyro_scale[0] = GYRO_SCALE_500_DPS;
			fd->gyro_scale[1] = GYRO_SCALE_500_DPS;
			fd->gyro_scale[2] = GYRO_SCALE_500_DPS;
			break;
		case SCALE_1000_DPS:
			gyro_mask |= (1<<4) | (0<<3);
			fd->gyro_scale[0] = GYRO_SCALE_1000_DPS;
			fd->gyro_scale[1] = GYRO_SCALE_1000_DPS;
			fd->gyro_scale[2] = GYRO_SCALE_1000_DPS;
			break;
		case SCALE_2000_DPS:
			gyro_mask |= (1<<4) | (1<<3);
			fd->gyro_scale[0] = GYRO_SCALE_2000_DPS;
			fd->gyro_scale[1] = GYRO_SCALE_2000_DPS;
			fd->gyro_scale[2] = GYRO_SCALE_2000_DPS;
			break;
	}
	// mpu9250_write_register(25, 19);
	// mpu9250_write_register(26, 6);
	mpu9250_write_register(MPU9250_GYRO_CONFIG, gyro_mask);
	return 0;
}

static int mpu9250_readAllData(struct imu_descriptor *fd)
{
	uint8_t sensor_raw_bytes[14];

	union {
		uint8_t in[2];
		int16_t output;
	} conv;

	int ret = mpu9250_read_multiple_registers(MPU9250_ACCEL_XOUT_H, sensor_raw_bytes, 14);

	if(ret < 14)
	{
		return -1;
	}

	conv.in[0] = sensor_raw_bytes[1];
	conv.in[1] = sensor_raw_bytes[0];
	fd->accRegisterDump[0] = conv.output;

	conv.in[0] = sensor_raw_bytes[3];
	conv.in[1] = sensor_raw_bytes[2];
	fd->accRegisterDump[1] = conv.output;

	conv.in[0] = sensor_raw_bytes[5];
	conv.in[1] = sensor_raw_bytes[4];
	fd->accRegisterDump[2] = conv.output;

	conv.in[0] = sensor_raw_bytes[9];
	conv.in[1] = sensor_raw_bytes[8];
	fd->gyroRegisterDump[0] = conv.output;

	conv.in[0] = sensor_raw_bytes[11];
	conv.in[1] = sensor_raw_bytes[10];
	fd->gyroRegisterDump[1] = conv.output;

	conv.in[0] = sensor_raw_bytes[13];
	conv.in[1] = sensor_raw_bytes[12];
	fd->gyroRegisterDump[2] = conv.output;
	return 0;
}

imu_desc mpu9250_spi = {
	1U, 1U, 0U, 0U,

	mpu9250_read_register,
	mpu9250_write_register,
	
	mpu9250_init,	
	mpu9250_setAccScale,
	mpu9250_setGyroScale,
	NULL,

	NULL,
	NULL,
	NULL,
	NULL,
	mpu9250_readAllData,
	
	{1.0f, 1.0f, 1.0f},
	{1.0f, 1.0f, 1.0f},
	{1.0f, 1.0f, 1.0f},

	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f},

	{MPU9250_ACC_X_SIGN, MPU9250_ACC_Y_SIGN, MPU9250_ACC_Z_SIGN},
	{MPU9250_GYRO_ROLL_SIGN, MPU9250_GYRO_PITCH_SIGN, MPU9250_GYRO_YAW_SIGN},
	{MPU9250_MAG_X_SIGN, MPU9250_MAG_Y_SIGN, MPU9250_MAG_Z_SIGN},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	0
};
