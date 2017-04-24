#include "imu_HAL.h"

static int32_t i2cReceiveByte_safe(i2cBASE_t *i2c)
{
	uint32_t timeout_counter = 0U;
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((i2c->STR & (uint32)I2C_RX_INT) == 0U)
    {
    	++timeout_counter;
    	if(timeout_counter > TIMEOUT_COUNTER_MAX)
    	{
    		i2cREG1->MDR |= (1<<11);				// Send STOP condition
    		return -1;
    	}
    } /* Wait */
/* USER CODE BEGIN (25) */
/* USER CODE END */
    return ((uint8)i2c->DRR);
}

static int32_t mpu9250_read_register(uint8_t reg_address)
{
	int32_t ret_val = 0U;

    i2cREG1->SAR = MPU9250_I2C_ADDR; 				// Use datasheet-spec'ed 7-bit address
        
    while(i2cREG1->MDR & (1<<11));                  // Check that stop condition has 
                                                    // been asserted previously
    i2cREG1->CNT = 1; 								// Set a 1-byte transfer
    i2cREG1->MDR = (1<<5 | 1<<9 | 1<<10 | 1<<13);
    i2cSendByte(i2cREG1, reg_address);
    while(!(i2cREG1->STR & (1<<4)));     
    i2cREG1->MDR = (1<<5 | 1<<10 | 1<<11 | 1<<13);
    ret_val = i2cReceiveByte_safe(i2cREG1);

	return ret_val;
}

static void mpu9250_write_register(uint8_t reg_address, uint8_t value)
{
		i2cREG1->SAR = MPU9250_I2C_ADDR;
		i2cREG1->CNT = 2;
		i2cREG1->MDR |= (1<<5 | 1<<9 | 1<<10 | 1<<13);

		while(!(i2cREG1->STR & (1<<4)));
		i2cREG1->DXR = reg_address;

		i2cREG1->MDR |= (1<<11);

		while(!(i2cREG1->STR & (1<<4)));
		i2cREG1->DXR = value;
}

static int32_t mpu9250_read_multiple_registers(uint8_t reg_base_address, uint8_t *buffer, uint8_t len)
{
	int32_t recv_byte = 0;

    i2cREG1->SAR = MPU9250_I2C_ADDR;				// Use datasheet-spec'ed 7-bit address
        
    while(i2cREG1->MDR & (1<<11));                  // Check that stop condition has 
                                                    // been asserted previously
    uint8_t i = 0U;
    i2cREG1->CNT = len+1; 							// len bytes to be transferred
    i2cREG1->MDR = (1<<5 | 1<<9 | 1<<10 | 1<<13);
    i2cSendByte(i2cREG1, reg_base_address);
    while(!(i2cREG1->STR & (1<<4)));     
    i2cREG1->MDR = (1<<5 | 1<<7 | 1<<10 | 1<<13);
    for(i=0U; i<len; ++i)
    {
    	recv_byte = i2cReceiveByte_safe(i2cREG1);
    	if(recv_byte < 0)
    	{
    		return i;
    	}
    	else
    	{
    		buffer[i] = (uint8_t)recv_byte;
    		if(i == len-2)
    		{
    			i2cREG1->MDR |= (1<<11); 			// Send stop before getting last byte in transfer
    		}
    	}
    }
    return i;
}

void imu_hal_init(void)
{
	i2cREG1->MDR = 0x00;

	i2cREG1->PSC = 7;             // Module clock frequency
	i2cREG1->CKL = 4;           // Low clock period: 4 for 400K, 32 for 100K (4 is EXPERIMENTAL!!)
	i2cREG1->CKH = 3;	          // High clock period: Ditto as above comment
	i2cREG1->OAR = 0x56;       // Set master address 0x56
	i2cREG1->IMR = 0x0;           // Interrupts disabled
	i2cREG1->CNT = 0;             // Initialize count to 0
	i2cREG1->SAR = 0;
	i2cREG1->PFNC = 0;             // Pins function as SDA and SCL pins
	i2cREG1->DIR = 0;             // Set I2C Functions
	i2cREG1->MDR |= 1<<10;        // Master
	i2cREG1->MDR |= 1<<5;         // Clear Reset
}

void initialize_imu(ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, imu_scaled_data_struct* buf)
{
	buf->acc_meas_scale = a;
	buf->gyro_meas_scale = g;
	buf->mag_meas_scale = m;

	#ifdef ACCEL_NO_LPF
		uint8_t accel_config2_mask = 0U;
	#endif
	#ifdef ACCEL_LPF_218_1_HZ
		uint8_t accel_config2_mask = 0 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_99_HZ
		uint8_t accel_config2_mask = 2 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_44_8_HZ
		uint8_t accel_config2_mask = 3 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_21_2_HZ
		uint8_t accel_config2_mask = 4 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_10_2_HZ
		uint8_t accel_config2_mask = 5 | (1<<3);
	#endif
	#ifdef ACCEL_LPF_5_05_HZ
		uint8_t accel_config2_mask = 6 | (1<<3);
	#endif
	#ifdef ACCEL_DEC2_420_HZ
		uint8_t accel_config_mask2 = 7 | (1<<3);
	#endif

	uint8_t accel_config_mask = 0U;
	uint8_t gyro_config_mask = 0U;

	switch(a)
	{
		case SCALE_2G:
			accel_config_mask |= (0<<4) | (0<<3);
			break;
		case SCALE_4G:
			accel_config_mask |= (0<<4) | (1<<3);
			break;
		case SCALE_8G:
			accel_config_mask |= (1<<4) | (0<<3);
			break;
		case SCALE_16G:
			accel_config_mask |= (1<<4) | (1<<3);
			break;
	}

	switch(g)
	{
		case SCALE_250_DPS:
			gyro_config_mask |= (0<<4) | (0<<3);
			break;
		case SCALE_500_DPS:
			gyro_config_mask |= (0<<4) | (1<<3);
			break;
		case SCALE_1000_DPS:
			gyro_config_mask |= (1<<4) | (0<<3);
			break;
		case SCALE_2000_DPS:
			gyro_config_mask |= (1<<4) | (1<<3);
			break;
	}
	// @Todo:
	// Magnetometer setup not yet implemented!! 

	mpu9250_write_register(MPU9250_ACCEL_CONFIG, accel_config_mask);
	mpu9250_write_register(MPU9250_ACCEL_CONFIG2, accel_config2_mask);
	mpu9250_write_register(MPU9250_GYRO_CONFIG, gyro_config_mask);
}

int get_raw_imu_data(imu_raw_data_struct* buffer)
{
	union {
		struct {
			int16_t x_accel_data;
			int16_t y_accel_data;
			int16_t z_accel_data;
			int16_t temp_sensor_data;
			int16_t x_gyro_data;
			int16_t y_gyro_data;
			int16_t z_gyro_data;
		} sensor_data_output;
		struct {
			uint8_t sensor_raw_bytes[14];
		} sensor_data_input;
	} convert_sensor_data;

	int ret = mpu9250_read_multiple_registers(MPU9250_ACCEL_XOUT_H, convert_sensor_data.sensor_data_input.sensor_raw_bytes, 14);

	if(ret < 14)
	{
		return -1;
	}

	uint8_t i = 0U;
	uint8_t temp_buf = 0U;

	for(i=0U; i<7U; ++i)
	{
		temp_buf = convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U];
		convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U] = convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U + 1U];
		convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U+1U] = temp_buf;
	}

	buffer->accel_data[0] = convert_sensor_data.sensor_data_output.x_accel_data;
	buffer->accel_data[1] = convert_sensor_data.sensor_data_output.y_accel_data;
	buffer->accel_data[2] = convert_sensor_data.sensor_data_output.z_accel_data;

	buffer->temp_sensor_data = convert_sensor_data.sensor_data_output.temp_sensor_data;

	buffer->gyro_data[0] = convert_sensor_data.sensor_data_output.x_gyro_data;
	buffer->gyro_data[1] = convert_sensor_data.sensor_data_output.y_gyro_data;
	buffer->gyro_data[2] = convert_sensor_data.sensor_data_output.z_gyro_data;
	return 0;
}