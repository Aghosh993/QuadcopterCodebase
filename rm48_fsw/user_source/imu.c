#include <imu.h>

int get_scaled_imu_data_no_bias_comp(imu_scaled_data_struct* buffer)
{
	imu_raw_data_struct data_buf;
	int read_res = get_raw_imu_data(&data_buf);

	if(read_res >= 0)
	{
		switch(buffer->acc_meas_scale)
		{
			case SCALE_2G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_2G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_2G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_2G;
				break;
			case SCALE_4G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_4G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_4G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_4G;
				break;
			case SCALE_8G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_8G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_8G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_8G;
				break;
			case SCALE_16G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_16G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_16G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_16G;
				break;
		}

		/*
			@TODO: Magnetometer not implemented yet!!
		 */

		switch(buffer->gyro_meas_scale)
		{
			case SCALE_250_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_250_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_250_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_250_DPS;
				break;
			case SCALE_500_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_500_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_500_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_500_DPS;
				break;
			#if defined USE_IMU_MPU9250_SPI
				case SCALE_1000_DPS:
					buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_1000_DPS;
					buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_1000_DPS;
					buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_1000_DPS;
					break;
			#endif
			case SCALE_2000_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_2000_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_2000_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_2000_DPS;
				break;
		}

		buffer->accel_data[0] *= (float)ACC_X_SIGN;
		buffer->accel_data[1] *= (float)ACC_Y_SIGN;
		buffer->accel_data[2] *= (float)ACC_Z_SIGN;

		/*
			@TODO: Magnetometer not implemented yet!!
		 */

		buffer->gyro_data[0] *= (float)GYRO_ROLL_SIGN;
		buffer->gyro_data[1] *= (float)GYRO_PITCH_SIGN;
		buffer->gyro_data[2] *= (float)GYRO_YAW_SIGN;

		buffer->temp_sensor_deg_c = (float)(data_buf.temp_sensor_data - (int16_t)TEMP_OFFSET) * (float)TEMP_SCALE + 21.0f;

		return 0; // Return READ_SUCCESS
	}
	return -1; // Failure
}

int get_scaled_imu_data(imu_scaled_data_struct* buffer)
{
	imu_raw_data_struct data_buf;
	int read_res = get_raw_imu_data(&data_buf);

	if(read_res >= 0)
	{
		switch(buffer->acc_meas_scale)
		{
			case SCALE_2G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_2G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_2G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_2G;
				break;
			case SCALE_4G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_4G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_4G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_4G;
				break;
			case SCALE_8G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_8G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_8G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_8G;
				break;
			case SCALE_16G:
				buffer->accel_data[0] = (float)data_buf.accel_data[ACCEL_AXIS_X] * (float)ACC_SCALE_16G;
				buffer->accel_data[1] = (float)data_buf.accel_data[ACCEL_AXIS_Y] * (float)ACC_SCALE_16G;
				buffer->accel_data[2] = (float)data_buf.accel_data[ACCEL_AXIS_Z] * (float)ACC_SCALE_16G;
				break;
		}

		/*
			@TODO: Magnetometer not implemented yet!!
		 */

		switch(buffer->gyro_meas_scale)
		{
			case SCALE_250_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_250_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_250_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_250_DPS;
				break;
			case SCALE_500_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_500_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_500_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_500_DPS;
				break;
			case SCALE_1000_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_1000_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_1000_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_1000_DPS;
				break;
			case SCALE_2000_DPS:
				buffer->gyro_data[0] = (float)data_buf.gyro_data[AXIS_ROLL] * (float)GYRO_SCALE_2000_DPS;
				buffer->gyro_data[1] = (float)data_buf.gyro_data[AXIS_PITCH] * (float)GYRO_SCALE_2000_DPS;
				buffer->gyro_data[2] = (float)data_buf.gyro_data[AXIS_YAW] * (float)GYRO_SCALE_2000_DPS;
				break;
		}

		buffer->accel_data[0] *= (float)ACC_X_SIGN;
		buffer->accel_data[1] *= (float)ACC_Y_SIGN;
		buffer->accel_data[2] *= (float)ACC_Z_SIGN;

		/*
			@TODO: Magnetometer not implemented yet!!
		 */

		buffer->gyro_data[AXIS_ROLL] *= (float)GYRO_ROLL_SIGN;
		buffer->gyro_data[AXIS_PITCH] *= (float)GYRO_PITCH_SIGN;
		buffer->gyro_data[AXIS_YAW] *= (float)GYRO_YAW_SIGN;

		buffer->temp_sensor_deg_c = (float)(data_buf.temp_sensor_data - (int16_t)TEMP_OFFSET) * (float)TEMP_SCALE + 21.0f;

		buffer->gyro_data[0] -= buffer->gyro_bias[0];
		buffer->gyro_data[1] -= buffer->gyro_bias[1];
		buffer->gyro_data[2] -= buffer->gyro_bias[2];

		buffer->accel_data[0] -= buffer->accel_bias[0];
		buffer->accel_data[1] -= buffer->accel_bias[1];
		buffer->accel_data[2] -= buffer->accel_bias[2];

		return 0; // Return READ_SUCCESS
	}
	return -1; // Failure
}
