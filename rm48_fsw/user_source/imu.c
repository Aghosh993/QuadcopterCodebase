#include <imu.h>

int get_scaled_imu_data(imu_scaled_data_struct* buffer)
{
	imu_raw_data_struct data_buf;
	int read_res = get_raw_imu_data(&data_buf);

	if(read_res >= 0)
	{
		switch(buffer->acc_meas_scale)
		{
			case SCALE_2G:
				buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)ACC_SCALE_2G;
				buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)ACC_SCALE_2G;
				buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)ACC_SCALE_2G;
				break;
			case SCALE_4G:
				buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)ACC_SCALE_4G;
				buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)ACC_SCALE_4G;
				buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)ACC_SCALE_4G;
				break;
			case SCALE_8G:
				buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)ACC_SCALE_8G;
				buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)ACC_SCALE_8G;
				buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)ACC_SCALE_8G;
				break;
			case SCALE_16G:
				buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)ACC_SCALE_16G;
				buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)ACC_SCALE_16G;
				buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)ACC_SCALE_16G;
				break;
		}

		/*
			@TODO: Magnetometer not implemented yet!!
		 */

		switch(buffer->gyro_meas_scale)
		{
			case SCALE_250_DPS:
				buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)GYRO_SCALE_250_DPS;
				buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)GYRO_SCALE_250_DPS;
				buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)GYRO_SCALE_250_DPS;
				break;
			case SCALE_500_DPS:
				buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)GYRO_SCALE_500_DPS;
				buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)GYRO_SCALE_500_DPS;
				buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)GYRO_SCALE_500_DPS;
				break;
			case SCALE_1000_DPS:
				buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)GYRO_SCALE_1000_DPS;
				buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)GYRO_SCALE_1000_DPS;
				buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)GYRO_SCALE_1000_DPS;
				break;
			case SCALE_2000_DPS:
				buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)GYRO_SCALE_2000_DPS;
				buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)GYRO_SCALE_2000_DPS;
				buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)GYRO_SCALE_2000_DPS;
				break;
		}

		buffer->accel_data[AXIS_X] *= (float)ACC_X_SIGN;
		buffer->accel_data[AXIS_Y] *= (float)ACC_Y_SIGN;
		buffer->accel_data[AXIS_Z] *= (float)ACC_Z_SIGN;

		/*
			@TODO: Magnetometer not implemented yet!!
		 */

		buffer->gyro_data[AXIS_X] *= (float)GYRO_X_SIGN;
		buffer->gyro_data[AXIS_Y] *= (float)GYRO_Y_SIGN;
		buffer->gyro_data[AXIS_Z] *= (float)GYRO_Z_SIGN;

		buffer->temp_sensor_deg_c = (float)(data_buf.temp_sensor_data - (int16_t)TEMP_OFFSET) * (float)TEMP_SCALE + 21.0f;
		return 0; // Return READ_SUCCESS
	}
	return -1; // Failure
}
