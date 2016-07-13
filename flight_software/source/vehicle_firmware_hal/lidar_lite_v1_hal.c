#include "lidar_lite_v1_hal.h"

void lidar_lite_i2c_bus_setup(void)
{
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_set_i2c_clock_hsi(I2C2);

	i2c_reset(I2C2);
	gpio_mode_setup(I2C2_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C2_SCL_PIN | I2C2_SDA_PIN);
	gpio_set_af(I2C2_GPIO, GPIO_AF4, I2C2_SCL_PIN | I2C2_SDA_PIN);

	i2c_peripheral_disable(I2C2);

	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C2);
	i2c_set_digital_filter(I2C2, I2C_CR1_DNF_DISABLED);

	//Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0] SCLH[7:0] SCLL[7:0]
	// in TIMINGR

	i2c_100khz_i2cclk8mhz(I2C2);

	//configure No-Stretch CR1 (only relevant in slave mode)
	i2c_enable_stretching(I2C2);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C2);
	i2c_peripheral_enable(I2C2);
}

uint8_t lidar_lite_i2c_read_byte(uint8_t reg)
{
	uint8_t return_value;
	read_i2c(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, reg, 1, &return_value);
	return return_value;
}
void lidar_lite_i2c_read_data(uint8_t start_reg, uint8_t size, uint8_t* data_buffer)
{
	read_i2c(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, start_reg | 0x80, size, data_buffer);
}

void lidar_lite_i2c_write_byte(uint8_t reg, uint8_t data_byte)
{
	write_i2c(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, reg, 1, &data_byte);
}

void lidar_lite_i2c_write_data(uint8_t start_reg, uint8_t size, uint8_t* data_buffer)
{
	write_i2c(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, start_reg, size, data_buffer);
}

lidar_lite_i2c_read_status lidar_lite_i2c_read_byte_with_timeout(uint8_t reg, uint8_t *data_read)
{
	read_i2c_with_timeout(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, reg, 1, data_read);
}
lidar_lite_i2c_read_status lidar_lite_i2c_read_data_with_timeout(uint8_t start_reg, uint8_t size, 
																	uint8_t* data_buffer)
{
	read_i2c_with_timeout(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, start_reg | 0x80, size, data_buffer);
}

lidar_lite_i2c_write_status lidar_lite_i2c_write_byte_with_timeout(uint8_t reg, uint8_t data_byte)
{
	write_i2c_with_timeout(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, reg, 1, &data_byte);
}

lidar_lite_i2c_write_status lidar_lite_i2c_write_data_with_timeout(uint8_t start_reg, uint8_t size, 
																	uint8_t* data_buffer)
{
	write_i2c_with_timeout(I2C2, LIDAR_LITE_I2C_ADDR_7BIT, start_reg, size, data_buffer);
}

/*
	Attempts to find sensor on I2C bus. Returns 0 on success, or -1 on reception of NACK
	(NOTE: NACK reception can indicate either lack of sensor present on bus, OR sensor busy due to
	calibration/measurement process.)
	Adapted from i2c_write() detailed at
	http://libopencm3.github.io/docs/latest/stm32f3/html/i2c_8c_source.html#l00411
	(i.e. from i2c.c driver in libopencm3):
 */

int lidar_lite_i2c_poll(void)
{
	/*
		Wait for I2C peripheral/bus to become free:
	 */
	while(i2c_busy(I2C2) == 1);
	while (i2c_is_start(I2C2) == 1);

	/*Setting transfer properties*/
	i2c_set_bytes_to_transfer(I2C2, 1);
	i2c_set_7bit_address(I2C2, LIDAR_LITE_I2C_ADDR_7BIT);
	i2c_set_write_transfer_dir(I2C2);
	i2c_enable_autoend(I2C2);

	/*start transfer*/
	i2c_send_start(I2C2);

	int wait = 1;
	int timeout_ctr = 0;

	while(wait==1)
	{
		if(i2c_transmit_int_status(I2C2))
		{
			wait = 0;
		}
		while(i2c_nack(I2C2))
		{
			++timeout_ctr;
			if(timeout_ctr>LIDAR_LITE_POLLING_TIMEOUT_LIMIT)
			{
				lidar_lite_i2c_reset();
				lidar_lite_i2c_bus_setup();
				return -1;
			}
		}
			++timeout_ctr;
			if(timeout_ctr>LIDAR_LITE_POLLING_TIMEOUT_LIMIT)
			{
				lidar_lite_i2c_reset();
				lidar_lite_i2c_bus_setup();
				return -1;
			}
	}
	/*
		We made it! Apparently this means we have received an ACK
	 */
	i2c_send_stop(I2C2);
	return 0;
}

void lidar_lite_i2c_reset(void)
{
	_disable_interrupts();

	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Set GPIO8 and 12 (in GPIO port E) to 'output push-pull'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C2_SCL_PIN | I2C2_SDA_PIN);

	gpio_set(GPIOA, I2C2_SDA_PIN);

	/*
		Clock out 8 high bits via SDA, to force all slaves to release I2C2 Bus:
	 */

	uint8_t i = 0U;
	uint8_t j = 0U;
	for(j = 0U; j < 16U; ++j)
	{
		gpio_toggle(GPIOA, I2C2_SDA_PIN);
		/*
			Approximately 50 us-wide between rising and falling edges of waveform, at 64 MHz SYSCLK
		 */
		for(i = 0U; i < 30U; ++i)
		{
			asm volatile("nop");
		}
	}
	gpio_set(GPIOA, I2C2_SCL_PIN);

	/*
		Delay by 10 ms before re-allocating GPIO pins to I2C2 alternate 
		function and returning:
	 */	

	for(i = 0U; i < 200U; ++i)
	{
		asm volatile("nop");
	}

	gpio_mode_setup(I2C2_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C2_SCL_PIN | I2C2_SDA_PIN);
	gpio_set_af(I2C2_GPIO, GPIO_AF4, I2C2_SCL_PIN | I2C2_SDA_PIN);

	_enable_interrupts();
}

lidar_lite_i2c_write_status write_i2c_with_timeout(uint32_t i2c, uint8_t i2c_addr, uint8_t reg, 
													uint8_t size,
	       											uint8_t *data)
{
	int wait;
	int i;
	int timeout_ctr = 0;
	int timeout_ctr2 = 0;
	while (i2c_busy(i2c) == 1)
	{
		if(advance_check_timeout_counter(&timeout_ctr) < 0)
		{
			return READ_TIMEOUT;
		}
	}

	timeout_ctr = 0;
	while (i2c_is_start(i2c) == 1)
	{
		if(advance_check_timeout_counter(&timeout_ctr) < 0)
		{
			return READ_TIMEOUT;
		}
	}
	/*Setting transfer properties*/
	i2c_set_bytes_to_transfer(i2c, size + 1);
	i2c_set_7bit_address(i2c, (i2c_addr & 0x7F));
	i2c_set_write_transfer_dir(i2c);
	i2c_enable_autoend(i2c);
	/*start transfer*/
	i2c_send_start(i2c);

	wait = true;
	timeout_ctr = 0;
	while (wait) {
		if(advance_check_timeout_counter(&timeout_ctr) < 0)
		{
			return READ_TIMEOUT;
		}

		if (i2c_transmit_int_status(i2c)) {
			wait = false;
		}
		timeout_ctr2 = 0;
		while (i2c_nack(i2c))
		{
			if(advance_check_timeout_counter(&timeout_ctr2) < 0)
			{
				return READ_TIMEOUT;
			}
		}
	}

	timeout_ctr = 0;

	i2c_send_data(i2c, reg);
	for (i = 0; i < size; i++) {
		wait = true;
		while (wait) {
			if(advance_check_timeout_counter(&timeout_ctr) < 0)
			{
				return READ_TIMEOUT;
			}

			if (i2c_transmit_int_status(i2c)) {
				wait = false;
			}
			timeout_ctr2 = 0;
			while (i2c_nack(i2c))
			{
				if(advance_check_timeout_counter(&timeout_ctr2) < 0)
				{
					return READ_TIMEOUT;
				}
			}
		}
		i2c_send_data(i2c, data[i]);
	}

	return WRITE_SUCCESS;
}

lidar_lite_i2c_read_status read_i2c_with_timeout(uint32_t i2c, uint8_t i2c_addr, uint8_t reg, 
													uint8_t size,
	      											uint8_t *data)
{
	int wait;
	int i;
	int timeout_ctr = 0;
	while (i2c_busy(i2c) == 1)
	{
		if(advance_check_timeout_counter(&timeout_ctr) < 0)
		{
			return READ_TIMEOUT;
		}
	}
	timeout_ctr = 0;
	while (i2c_is_start(i2c) == 1)
	{
		if(advance_check_timeout_counter(&timeout_ctr) < 0)
		{
			return READ_TIMEOUT;
		}
	}
	/*Setting transfer properties*/
	i2c_set_bytes_to_transfer(i2c, 1);
	i2c_set_7bit_address(i2c, i2c_addr);
	i2c_set_write_transfer_dir(i2c);
	i2c_disable_autoend(i2c);
	/*start transfer*/
	i2c_send_start(i2c);

	wait = true;
	timeout_ctr = 0;
	int timeout_ctr2 = 0;

	while (wait) {
		if(advance_check_timeout_counter(&timeout_ctr2) < 0)
		{
			return READ_TIMEOUT;
		}

		if (i2c_transmit_int_status(i2c)) {
			wait = false;
		}
		while (i2c_nack(i2c)) /* Some error */
		{
			if(advance_check_timeout_counter(&timeout_ctr) < 0)
			{
				return READ_TIMEOUT;
			}
		}
	}
	i2c_send_data(i2c, reg);

	timeout_ctr = 0;
	while (i2c_is_start(i2c) == 1)
	{
		if(advance_check_timeout_counter(&timeout_ctr) < 0)
		{
			return READ_TIMEOUT;
		}
	}
	/*Setting transfer properties*/
	i2c_set_bytes_to_transfer(i2c, size);
	i2c_set_7bit_address(i2c, i2c_addr);
	i2c_set_read_transfer_dir(i2c);
	i2c_enable_autoend(i2c);
	/*start transfer*/
	i2c_send_start(i2c);

	timeout_ctr = 0;
	for (i = 0; i < size; i++) {
		while (i2c_received_data(i2c) == 0)
		{
			if(advance_check_timeout_counter(&timeout_ctr) < 0)
			{
				return READ_TIMEOUT;
			}
		}
		data[i] = i2c_get_data(i2c);
	}

	return READ_SUCCESS;
}

int advance_check_timeout_counter(int *timeout_counter)
{
	++(*timeout_counter);
	if(*timeout_counter > LIDAR_LITE_POLLING_TIMEOUT_LIMIT)
	{
		return -1;
	}
	return 0;
}