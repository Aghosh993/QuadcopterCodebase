#include <imu_hal.h>

/*
	Sets up I2C peripheral:
	Taken from libopencm3 examples under stm32/f3/f3discovery/i2c:
 */

static void i2c_setup(void)
{
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_set_i2c_clock_hsi(I2C1);

	i2c_reset(I2C1);
	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6| GPIO7);
	i2c_peripheral_disable(I2C1);
	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
	//Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0] SCLH[7:0] SCLL[7:0]
	// in TIMINGR
	/*
		Cannot use 100 KHz default setup from Libopencm3 examples, we want 400 KHz.
		See setup below commented-out line:

		(Edit: defaults from 400 KHz didn't work with LSM303DLHC magnetometer
		and led to magnetometer values not updating. Probably timing issue due to excessive clock)
	 */
	// i2c_100khz_i2cclk8mhz(I2C1);

	i2c_set_prescaler(I2C1,1);
	i2c_set_scl_low_period(I2C1, 0x02);
	i2c_set_scl_high_period(I2C1, 0x03);
	i2c_set_data_hold_time(I2C1, 0x01);
	i2c_set_data_setup_time(I2C1, 0x03);

	//configure No-Stretch CR1 (only relevant in slave mode)
	i2c_enable_stretching(I2C1);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C1);
	i2c_peripheral_enable(I2C1);
}

void imu_i2c_bus_clear(void)
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

/*
	Sets up SPI peripheral and CS (Chip Select) lines:
	(Taken from libopencm3 examples under stm32/f3/f3discovery/spi)
 */

static void spi_setup(void)
{
	/*
		Route peripheral clocks:
	 */
	rcc_periph_clock_enable(RCC_SPI1);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOA);
	/* For spi mode select on the l3gd20 */
	rcc_periph_clock_enable(RCC_GPIOE);

	/*
		Set up CS lines on GPIO module, and set alternate function
		for GPIO lines dedicated to SPI:
	 */
	/* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	/* Start with spi communication disabled */
	gpio_set(L3GD20_CS_GPIO);

	/* Setup GPIO pins for AF5 for SPI1 signals. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

	/*
		SPI initialization code:
	 */
	spi_set_master_mode(SPI1);
	spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_32);
	spi_set_clock_polarity_0(SPI1);
	spi_set_clock_phase_0(SPI1);
	spi_set_full_duplex_mode(SPI1);
	spi_set_unidirectional_mode(SPI1); /* bidirectional but in 3-wire */
	spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
	spi_enable_software_slave_management(SPI1);
	spi_send_msb_first(SPI1);
	spi_set_nss_high(SPI1);
	spi_fifo_reception_threshold_8bit(SPI1);
	SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI1);
}

static void spi_write_to_l3gd20_register(uint32_t cs_gpio_port, uint16_t cs_pin, uint8_t reg, uint8_t data)
{
	/*
		Select L3GD20 for SPI transaction:
	 */
	gpio_clear(cs_gpio_port, cs_pin);
	/*
		Send register we wish to write to:
	 */
	spi_send8(SPI1, reg);
	/*
		Read data that has been clocked back in:
	 */
	uint8_t dummy_byte = spi_read8(SPI1);
	/*
		Send register contents:
	 */
	spi_send8(SPI1, data);
	/*
		Read data that has been clocked back in:
	 */
	dummy_byte = spi_read8(SPI1);
	gpio_set(cs_gpio_port, cs_pin);
}

/*
	Function to read a sequence of bytes from l3gd20.
 */
static void spi_read_from_l3gd20_register(uint32_t cs_gpio_port, uint16_t cs_pin, uint8_t start_reg, uint8_t len, uint8_t *data_buffer)
{
	uint8_t dummy_byte = 0x00;
	uint8_t i = 0U;
	/*
		Select L3GD20 for SPI transaction:
	 */
	gpio_clear(cs_gpio_port, cs_pin);
	/*
		Send register we wish to start read sequence from, logical
		OR with masks to enable address auto-increment and read mode:
	 */
	spi_send8(SPI1, start_reg | L3GD20_GYRO_READ_MASK | L3GD20_GYRO_SEQUENTIAL_READ_MASK);
	/*
		Read dummy data that has been clocked back in:
	 */
	spi_read8(SPI1);
	/*
		Read data:
	 */
	for(i=0U; i<len; ++i)
	{
		/*
			Send zeroes and keep clocking data in until we fill the data buffer:
		 */
		spi_send8(SPI1, dummy_byte);
		data_buffer[i] = spi_read8(SPI1);
	}
	gpio_set(cs_gpio_port, cs_pin);
}

void initialize_imu(ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, imu_scaled_data_struct* buf)
{
	buf->acc_meas_scale = a;
	buf->gyro_meas_scale = g;
	buf->mag_meas_scale = m;

	uint8_t i = 0U;

	for(i=0U;i<3U;++i)
	{
		buf->accel_data[i]=0.0f;
		buf->gyro_data[i]=0.0f;
		buf->magnetometer_data[i]=0.0f;
	}

	i2c_setup();
	spi_setup();

	uint8_t acc_ctrl_reg1_mask = LSM303_ACC_XEN | LSM303_ACC_YEN | LSM303_ACC_ZEN |
									LSM303_ACC_ODR_1344HZ_MASK;

	uint8_t acc_ctrl_reg4_mask = LSM303_ACC_HIGHRES_MODE_MASK;

	switch(a)
	{
		case SCALE_2G:
			acc_ctrl_reg4_mask |= LSM303_ACC_2G_MASK;
			break;
		case SCALE_4G:
			acc_ctrl_reg4_mask |= LSM303_ACC_4G_MASK;
			break;
		case SCALE_8G:
			acc_ctrl_reg4_mask |= LSM303_ACC_8G_MASK;
			break;
		case SCALE_16G:
			acc_ctrl_reg4_mask |= LSM303_ACC_16G_MASK;
			break;
	}

	uint8_t mag_cra_mask = LSM303_MAG_DO_220HZ_MASK;
	
	uint8_t mag_crb_mask = 0U;

	switch(m)
	{
		case SCALE_1POINT3_GAUSS:
			mag_crb_mask |= LSM303_MAG_1POINT3_GAUSS_MASK;
			break;
		case SCALE_1POINT9_GAUSS:
			mag_crb_mask |= LSM303_MAG_1POINT9_GAUSS_MASK;
			break;
		case SCALE_2POINT5_GAUSS:
			mag_crb_mask |= LSM303_MAG_2POINT5_GAUSS_MASK;
			break;
		case SCALE_4POINT0_GAUSS:
			mag_crb_mask |= LSM303_MAG_4POINT0_GAUSS_MASK;
			break;
		case SCALE_4POINT7_GAUSS:
			mag_crb_mask |= LSM303_MAG_4POINT7_GAUSS_MASK;
			break;
		case SCALE_5POINT6_GAUSS:
			mag_crb_mask |= LSM303_MAG_5POINT6_GAUSS_MASK;
			break;
		case SCALE_8POINT1_GAUSS:
			mag_crb_mask |= LSM303_MAG_8POINT1_GAUSS_MASK;
			break;
	}

	uint8_t mag_mr_mask = LSM303_MAG_CONTINUOUS_CONV_MASK;

	/*
		Write initialization data to accelerometer:
	 */

	write_i2c(I2C1, ADDR_LSM303DLHC_ACC, LSM303_CTRL_REG1_A, 1, &acc_ctrl_reg1_mask);
	write_i2c(I2C1, ADDR_LSM303DLHC_ACC, LSM303_CTRL_REG4_A, 1, &acc_ctrl_reg4_mask);

	/*
		Write initialization data to magnetometer:
	 */
	write_i2c(I2C1, ADDR_LSM303DLHC_MAG, LSM303_CRA_REG_M, 	1, 	&mag_cra_mask);
	write_i2c(I2C1, ADDR_LSM303DLHC_MAG, LSM303_CRB_REG_M, 	1, 	&mag_crb_mask);
	write_i2c(I2C1, ADDR_LSM303DLHC_MAG, LSM303_MR_REG_M, 	1, 	&mag_mr_mask);

	/*
		Gyro intialization:
	 */

	uint8_t gyro_ctrl_reg1_mask = L3GD20_GYRO_XEN | L3GD20_GYRO_YEN | L3GD20_GYRO_ZEN | LSGD20_PD_NORMAL_MODE;

	uint8_t gyro_ctrl_reg4_mask = 0U;

	switch(g)
	{
		case SCALE_250_DPS:
			gyro_ctrl_reg4_mask = L3GD20_GYRO_250DPS_MASK;
			break;
		case SCALE_500_DPS:
			gyro_ctrl_reg4_mask = L3GD20_GYRO_500DPS_MASK;
			break;
		case SCALE_2000_DPS:
			gyro_ctrl_reg4_mask = L3GD20_GYRO_2000DPS_MASK;
			break;
	}

	spi_write_to_l3gd20_register(L3GD20_CS_GPIO, L3GD20_GYRO_CTRL_REG1, gyro_ctrl_reg1_mask);
	spi_write_to_l3gd20_register(L3GD20_CS_GPIO, L3GD20_GYRO_CTRL_REG4, gyro_ctrl_reg4_mask);
}

void get_raw_imu_data(imu_raw_data_struct* buffer)
{
	uint8_t acc_data_raw[6] 	= 	{0,0,0,0,0,0};
	uint8_t mag_data_raw[6] 	= 	{0,0,0,0,0,0};
	uint8_t gyro_data_raw[6] 	= 	{0,0,0,0,0,0};

	uint8_t tmp;

	read_i2c(I2C1, ADDR_LSM303DLHC_ACC, LSM303_ACC_OUT_X_L | 0x80, 6, acc_data_raw);
	read_i2c(I2C1, ADDR_LSM303DLHC_MAG, LSM303_MAG_OUT_X_H, 6, mag_data_raw);

	buffer->accel_data[AXIS_X] = ((int16_t)(acc_data_raw[2] | acc_data_raw[3] << 8))>>4;
	buffer->accel_data[AXIS_Y] = ((int16_t)(acc_data_raw[0] | acc_data_raw[1] << 8))>>4;
	buffer->accel_data[AXIS_Z] = ((int16_t)(acc_data_raw[4] | acc_data_raw[5] << 8))>>4;

	buffer->magnetometer_data[AXIS_X] = ((int16_t)(mag_data_raw[1] | mag_data_raw[0] << 8));//>>4;
	buffer->magnetometer_data[AXIS_Y] = ((int16_t)(mag_data_raw[5] | mag_data_raw[4] << 8));//>>4;
	buffer->magnetometer_data[AXIS_Z] = ((int16_t)(mag_data_raw[3] | mag_data_raw[2] << 8));//>>4;

	spi_read_from_l3gd20_register(L3GD20_CS_GPIO, L3GD20_GYRO_OUT_X_L, 6, gyro_data_raw);

	buffer->gyro_data[AXIS_X] = (int16_t)(gyro_data_raw[0] | gyro_data_raw[1] << 8);
	buffer->gyro_data[AXIS_Y] = (int16_t)(gyro_data_raw[2] | gyro_data_raw[3] << 8);
	buffer->gyro_data[AXIS_Z] = (int16_t)(gyro_data_raw[4] | gyro_data_raw[5] << 8);
}
