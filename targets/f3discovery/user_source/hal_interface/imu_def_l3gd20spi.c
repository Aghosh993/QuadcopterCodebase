#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "hal_common_includes.h"
#include "imu_hal.h"

// L3GD20 registers and methods:

#define L3GD20_AXIS_ROLL	1
#define L3GD20_AXIS_PITCH	0
#define L3GD20_AXIS_YAW		2

#define L3GD20_GYRO_ROLL_SIGN	-1.0f
#define L3GD20_GYRO_PITCH_SIGN	-1.0f
#define L3GD20_GYRO_YAW_SIGN	-1.0f

#define L3GD20_GYRO_CTRL_REG1	0x20

#define L3GD20_GYRO_XEN			(1<<1)
#define L3GD20_GYRO_YEN			(1<<0)
#define L3GD20_GYRO_ZEN			(1<<2)

#define LSGD20_PD_NORMAL_MODE	(1<<3)

#define L3GD20_GYRO_CTRL_REG4	0x23

#define L3GD20_GYRO_250DPS_MASK		(0<<4) | (0<<5)
#define L3GD20_GYRO_500DPS_MASK		(1<<4) | (0<<5)
#define L3GD20_GYRO_2000DPS_MASK	(1<<4) | (1<<5)

#define L3GD20_GYRO_SEQUENTIAL_RW_MASK			(1<<6)
#define L3GD20_GYRO_READ_MASK					(1<<7)

#define L3GD20_GYRO_OUT_X_L			0x28
#define L3GD20_GYRO_OUT_X_H			0x29

#define L3GD20_GYRO_OUT_Y_L			0x2A
#define L3GD20_GYRO_OUT_Y_H			0x2B

#define L3GD20_GYRO_OUT_Z_L			0x2C
#define L3GD20_GYRO_OUT_Z_H			0x2D

/*
	Gyro conversion factor results in degrees/s units:
 */

#define L3GD20_GYRO_SCALE_250_DPS			0.00875f
#define L3GD20_GYRO_SCALE_500_DPS			0.01750f
#define L3GD20_GYRO_SCALE_2000_DPS			0.07000f

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

static int spi_write_to_l3gd20_register(uint8_t reg, uint8_t data)
{
	/*
		Select L3GD20 for SPI transaction:
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

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
		De-select the L3GD20 for SPI transaction:
	 */

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	return 0;
}

/*
	Function to read a sequence of bytes from l3gd20.
 */
static int spi_read_from_l3gd20_register(uint8_t start_reg, uint8_t len, uint8_t *data_buffer)
{
	uint8_t dummy_byte = 0U;
	uint8_t i = 0U;

	/*
		Select L3GD20 for SPI transaction:
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	/*
		Send register we wish to start read sequence from, logical
		OR with masks to enable address auto-increment and read mode:
	 */
	uint8_t receive_cmd = start_reg | L3GD20_GYRO_READ_MASK;

	if(len > 1)
	{
		receive_cmd |= L3GD20_GYRO_SEQUENTIAL_RW_MASK;
	}
	
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
		De-select the L3GD20 for SPI transaction:
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return 0;
}

static void l3gd20_read_register(uint8_t reg_address, uint8_t *buf)
{
	spi_read_from_l3gd20_register(reg_address, 1U, buf);
}

static void l3gd20_write_register(uint8_t reg_address, uint8_t value)
{
	spi_write_to_l3gd20_register(reg_address, value);
}

static int32_t l3gd20_read_multiple_registers(uint8_t reg_base_address, uint8_t *buffer, uint8_t len)
{
	spi_read_from_l3gd20_register(reg_base_address, len, buffer);
	return len;
}

static volatile uint8_t accel_mask1, accel_mask2;
static volatile uint8_t gyro_mask;
static volatile uint8_t mag_mask;

static void l3gd20_init(void)
{
	spi_setup(BAUD_5M);
	uint8_t gyro_ctrl_reg1_mask = L3GD20_GYRO_XEN | L3GD20_GYRO_YEN | L3GD20_GYRO_ZEN | LSGD20_PD_NORMAL_MODE;
	spi_write_to_l3gd20_register(L3GD20_GYRO_CTRL_REG1, gyro_ctrl_reg1_mask);
}

static int l3gd20_setGyroScale(struct imu_descriptor *fd, GYRO_SCALE gs)
{
	gyro_mask = 0U;
	switch(gs)
	{
		case SCALE_250_DPS:
			gyro_mask |= L3GD20_GYRO_250DPS_MASK;
			fd->gyro_scale[0] = L3GD20_GYRO_SCALE_250_DPS;
			fd->gyro_scale[1] = L3GD20_GYRO_SCALE_250_DPS;
			fd->gyro_scale[2] = L3GD20_GYRO_SCALE_250_DPS;
			break;
		case SCALE_500_DPS:
			gyro_mask |= L3GD20_GYRO_250DPS_MASK;
			fd->gyro_scale[0] = L3GD20_GYRO_SCALE_500_DPS;
			fd->gyro_scale[1] = L3GD20_GYRO_SCALE_500_DPS;
			fd->gyro_scale[2] = L3GD20_GYRO_SCALE_500_DPS;
			break;
		case SCALE_2000_DPS:
			gyro_mask |= L3GD20_GYRO_250DPS_MASK;
			fd->gyro_scale[0] = L3GD20_GYRO_SCALE_2000_DPS;
			fd->gyro_scale[1] = L3GD20_GYRO_SCALE_2000_DPS;
			fd->gyro_scale[2] = L3GD20_GYRO_SCALE_2000_DPS;
			break;
	}
	spi_write_to_l3gd20_register(L3GD20_GYRO_CTRL_REG4, gyro_mask);
	return 0;
}

static int l3gd20_readAllData(struct imu_descriptor *fd)
{
	uint8_t sensor_raw_bytes[6];

	union {
		uint8_t in[2];
		int16_t output;
	} conv;

	if(spi_read_from_l3gd20_register(L3GD20_GYRO_OUT_X_L, 6, sensor_raw_bytes) < 0)
	{
		return -1;
	}

	conv.in[0] = sensor_raw_bytes[0];
	conv.in[1] = sensor_raw_bytes[1];
	fd->gyroRegisterDump[L3GD20_AXIS_ROLL] = conv.output;

	conv.in[0] = sensor_raw_bytes[2];
	conv.in[1] = sensor_raw_bytes[3];
	fd->gyroRegisterDump[L3GD20_AXIS_PITCH] = conv.output;

	conv.in[0] = sensor_raw_bytes[4];
	conv.in[1] = sensor_raw_bytes[5];
	fd->gyroRegisterDump[L3GD20_AXIS_YAW] = conv.output;

	return 0;
}

imu_desc l3gd20_spi = {
	0U, 1U, 0U, 0U,

	l3gd20_read_register,
	l3gd20_write_register,
	
	l3gd20_init,	
	NULL,
	l3gd20_setGyroScale,
	NULL,

	NULL,
	NULL,
	NULL,
	NULL,
	l3gd20_readAllData,
	
	{1.0f, 1.0f, 1.0f},
	{1.0f, 1.0f, 1.0f},
	{1.0f, 1.0f, 1.0f},

	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f},

	{0.0f, 0.0f, 0.0f},
	{L3GD20_GYRO_ROLL_SIGN, L3GD20_GYRO_PITCH_SIGN, L3GD20_GYRO_YAW_SIGN},
	{0.0f, 0.0f, 0.0f},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	0
};
