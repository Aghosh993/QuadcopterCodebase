#ifndef LIDAR_LITE_HAL_V1_H
#define LIDAR_LITE_HAL_V1_H		1

#include <hal_common_includes.h>

#define I2C2_GPIO		GPIOA

#define I2C2_SCL_PIN	GPIO9
#define I2C2_SDA_PIN	GPIO10

#define LIDAR_LITE_I2C_ADDR_7BIT			0x62	// Effective 7-bit I2C device address of LIDAR-Lite

#define LIDAR_LITE_COMMAND_CONTROL_REG		0x00
#define LIDAR_LITE_HEIGHT_DATA_START_REG	0x0f

#define LIDAR_LITE_DC_COMP_CONV_MASK		0x04

/*
	Permits about 5.45 ms delay before triggering I2C timout:
 */
#define	LIDAR_LITE_POLLING_TIMEOUT_LIMIT	50000 	// Defined as number of CPU cycles to wait for ACK from sensor

typedef enum {
	READ_SUCCESS,
	READ_TIMEOUT
} lidar_lite_i2c_read_status;

typedef enum {
	WRITE_SUCCESS,
	WRITE_TIMEOUT
} lidar_lite_i2c_write_status;

void 	lidar_lite_i2c_bus_setup(void);
int 	lidar_lite_i2c_poll(void);
void 	lidar_lite_i2c_setup(void);

uint8_t lidar_lite_i2c_read_byte(uint8_t reg);
void 	lidar_lite_i2c_read_data(uint8_t start_reg, uint8_t size, uint8_t* data_buffer);

void lidar_lite_i2c_write_byte(uint8_t reg, uint8_t data_byte);
void lidar_lite_i2c_write_data(uint8_t start_reg, uint8_t size, uint8_t* data_buffer);

void lidar_lite_i2c_reset(void);

int advance_check_timeout_counter(int *timeout_counter);
lidar_lite_i2c_read_status read_i2c_with_timeout(uint32_t i2c, uint8_t i2c_addr, uint8_t reg, 
													uint8_t size,
	      											uint8_t *data);
lidar_lite_i2c_write_status write_i2c_with_timeout(uint32_t i2c, uint8_t i2c_addr, uint8_t reg, 
													uint8_t size,
	       											uint8_t *data);

lidar_lite_i2c_read_status lidar_lite_i2c_read_byte_with_timeout(uint8_t reg, uint8_t *data_read);
lidar_lite_i2c_read_status lidar_lite_i2c_read_data_with_timeout(uint8_t start_reg, uint8_t size, 
																	uint8_t* data_buffer);
lidar_lite_i2c_write_status lidar_lite_i2c_write_byte_with_timeout(uint8_t reg, uint8_t data_byte);
lidar_lite_i2c_write_status lidar_lite_i2c_write_data_with_timeout(uint8_t start_reg, uint8_t size, 
																	uint8_t* data_buffer);
#endif