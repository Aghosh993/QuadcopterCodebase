#include "imu_hal.h"

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

static void do_bias_calculation(imu_scaled_data_struct *imu_data)
{
	uint16_t i = 0U;

	float roll_gyro_running_sum = 0.0f;
	float pitch_gyro_running_sum = 0.0f;
	float yaw_gyro_running_sum = 0.0f;

	float x_accelerometer_running_sum = 0.0f;
	float y_accelerometer_running_sum = 0.0f;
	float z_accelerometer_running_sum = 0.0f;

	for(i=0U; i<BIAS_CALC_NUM_SAMPLES; ++i)
	{
		get_scaled_imu_data(imu_data);

		roll_gyro_running_sum += imu_data->gyro_data[AXIS_ROLL];
		pitch_gyro_running_sum += imu_data->gyro_data[AXIS_PITCH];
		yaw_gyro_running_sum += imu_data->gyro_data[AXIS_YAW];

		x_accelerometer_running_sum += imu_data->accel_data[ACCEL_AXIS_X];
		y_accelerometer_running_sum += imu_data->accel_data[ACCEL_AXIS_Y];
		z_accelerometer_running_sum += imu_data->accel_data[ACCEL_AXIS_Z];

		timekeeper_delay((uint16_t)BIAS_CALC_SAMPLE_DT_MS);
	}
	imu_data->gyro_bias[0] = (roll_gyro_running_sum/(float)BIAS_CALC_NUM_SAMPLES) - (float)GYRO_Y_IDEAL_READING;
	imu_data->gyro_bias[1] = (pitch_gyro_running_sum/(float)BIAS_CALC_NUM_SAMPLES) - (float)GYRO_X_IDEAL_READING;
	imu_data->gyro_bias[2] = (yaw_gyro_running_sum/(float)BIAS_CALC_NUM_SAMPLES) - (float)GYRO_Z_IDEAL_READING;

	imu_data->accel_bias[0] = (x_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES) - (float)ACCELEROMETER_X_IDEAL_READING;
	imu_data->accel_bias[1] = (y_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES) - (float)ACCELEROMETER_Y_IDEAL_READING;
	imu_data->accel_bias[2] = (z_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES) - (float)ACCELEROMETER_Z_IDEAL_READING;
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

int initialize_imu(ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, imu_scaled_data_struct* buf)
{
	imu_hal_init();
	
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

	do_bias_calculation(buf);
	return 0;
}

// int get_raw_imu_data(imu_raw_data_struct* buffer)
// {
// 	union {
// 		struct {
// 			int16_t x_accel_data;
// 			int16_t y_accel_data;
// 			int16_t z_accel_data;
// 			int16_t temp_sensor_data;
// 			int16_t x_gyro_data;
// 			int16_t y_gyro_data;
// 			int16_t z_gyro_data;
// 		} sensor_data_output;
// 		struct {
// 			uint8_t sensor_raw_bytes[14];
// 		} sensor_data_input;
// 	} convert_sensor_data;

// 	int ret = mpu9250_read_multiple_registers(MPU9250_ACCEL_XOUT_H, convert_sensor_data.sensor_data_input.sensor_raw_bytes, 14);

// 	if(ret < 14)
// 	{
// 		return -1;
// 	}

// 	uint8_t i = 0U;
// 	uint8_t temp_buf = 0U;

// 	for(i=0U; i<7U; ++i)
// 	{
// 		temp_buf = convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U];
// 		convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U] = convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U + 1U];
// 		convert_sensor_data.sensor_data_input.sensor_raw_bytes[i*2U+1U] = temp_buf;
// 	}

// 	buffer->accel_data[0] = convert_sensor_data.sensor_data_output.x_accel_data;
// 	buffer->accel_data[1] = convert_sensor_data.sensor_data_output.y_accel_data;
// 	buffer->accel_data[2] = convert_sensor_data.sensor_data_output.z_accel_data;

// 	buffer->temp_sensor_data = convert_sensor_data.sensor_data_output.temp_sensor_data;

// 	buffer->gyro_data[0] = convert_sensor_data.sensor_data_output.x_gyro_data;
// 	buffer->gyro_data[1] = convert_sensor_data.sensor_data_output.y_gyro_data;
// 	buffer->gyro_data[2] = convert_sensor_data.sensor_data_output.z_gyro_data;
// 	return 0;
// }

int get_raw_imu_data(imu_raw_data_struct* buffer)
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
	buffer->accel_data[0] = conv.output;

	conv.in[0] = sensor_raw_bytes[3];
	conv.in[1] = sensor_raw_bytes[2];
	buffer->accel_data[1] = conv.output;
	
	conv.in[0] = sensor_raw_bytes[5];
	conv.in[1] = sensor_raw_bytes[4];
	buffer->accel_data[2] = conv.output;

	conv.in[0] = sensor_raw_bytes[7];
	conv.in[1] = sensor_raw_bytes[6];
	buffer->temp_sensor_data = conv.output;

	conv.in[0] = sensor_raw_bytes[9];
	conv.in[1] = sensor_raw_bytes[8];
	buffer->gyro_data[0] = conv.output;

	conv.in[0] = sensor_raw_bytes[11];
	conv.in[1] = sensor_raw_bytes[10];
	buffer->gyro_data[1] = conv.output;

	conv.in[0] = sensor_raw_bytes[13];
	conv.in[1] = sensor_raw_bytes[12];
	buffer->gyro_data[2] = conv.output;
	return 0;
}