#include <math.h>

#include <imu.h>

void get_scaled_imu_data(imu_scaled_data_struct* buffer)
{
	imu_raw_data_struct data_buf;
	get_raw_imu_data(&data_buf);

	switch(buffer->acc_meas_scale)
	{
		case SCALE_2G:
			buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)LSM303_ACC_SCALE_2G;
			buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)LSM303_ACC_SCALE_2G;
			buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)LSM303_ACC_SCALE_2G;
			break;
		case SCALE_4G:
			buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)LSM303_ACC_SCALE_4G;
			buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)LSM303_ACC_SCALE_4G;
			buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)LSM303_ACC_SCALE_4G;
			break;
		case SCALE_8G:
			buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)LSM303_ACC_SCALE_8G;
			buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)LSM303_ACC_SCALE_8G;
			buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)LSM303_ACC_SCALE_8G;
			break;
		case SCALE_16G:
			buffer->accel_data[AXIS_X] = (float)data_buf.accel_data[AXIS_X] * (float)LSM303_ACC_SCALE_16G;
			buffer->accel_data[AXIS_Y] = (float)data_buf.accel_data[AXIS_Y] * (float)LSM303_ACC_SCALE_16G;
			buffer->accel_data[AXIS_Z] = (float)data_buf.accel_data[AXIS_Z] * (float)LSM303_ACC_SCALE_16G;
			break;
	}

	switch(buffer->mag_meas_scale)
	{
		case SCALE_1POINT3_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_1POINT3_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_1POINT3_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_1POINT3_GAUSS_Z;
			break;
		case SCALE_1POINT9_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_1POINT9_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_1POINT9_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_1POINT9_GAUSS_Z;
			break;
		case SCALE_2POINT5_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_2POINT5_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_2POINT5_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_2POINT5_GAUSS_Z;
			break;
		case SCALE_4POINT0_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_4POINT0_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_4POINT0_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_4POINT0_GAUSS_Z;
			break;
		case SCALE_4POINT7_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_4POINT7_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_4POINT7_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_4POINT7_GAUSS_Z;
			break;
		case SCALE_5POINT6_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_5POINT6_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_5POINT6_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_5POINT6_GAUSS_Z;
			break;
		case SCALE_8POINT1_GAUSS:
			buffer->magnetometer_data[AXIS_X] = (float)data_buf.magnetometer_data[AXIS_X] * (float)LSM303_MAG_SCALE_8POINT1_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Y] = (float)data_buf.magnetometer_data[AXIS_Y] * (float)LSM303_MAG_SCALE_8POINT1_GAUSS_XY;
			buffer->magnetometer_data[AXIS_Z] = (float)data_buf.magnetometer_data[AXIS_Z] * (float)LSM303_MAG_SCALE_8POINT1_GAUSS_Z;
			break;
	}

	switch(buffer->gyro_meas_scale)
	{
		case SCALE_250_DPS:
			buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)L3GD20_GYRO_SCALE_250_DPS;
			buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)L3GD20_GYRO_SCALE_250_DPS;
			buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)L3GD20_GYRO_SCALE_250_DPS;
			break;
		case SCALE_500_DPS:
			buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)L3GD20_GYRO_SCALE_500_DPS;
			buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)L3GD20_GYRO_SCALE_500_DPS;
			buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)L3GD20_GYRO_SCALE_500_DPS;
			break;
		case SCALE_2000_DPS:
			buffer->gyro_data[AXIS_X] = (float)data_buf.gyro_data[AXIS_X] * (float)L3GD20_GYRO_SCALE_2000_DPS;
			buffer->gyro_data[AXIS_Y] = (float)data_buf.gyro_data[AXIS_Y] * (float)L3GD20_GYRO_SCALE_2000_DPS;
			buffer->gyro_data[AXIS_Z] = (float)data_buf.gyro_data[AXIS_Z] * (float)L3GD20_GYRO_SCALE_2000_DPS;
			break;
	}

	buffer->accel_data[AXIS_X] *= (float)ACC_X_SIGN;
	buffer->accel_data[AXIS_Y] *= (float)ACC_Y_SIGN;
	buffer->accel_data[AXIS_Z] *= (float)ACC_Z_SIGN;

	buffer->magnetometer_data[AXIS_X] *= (float)MAG_X_SIGN;
	buffer->magnetometer_data[AXIS_Y] *= (float)MAG_Y_SIGN;
	buffer->magnetometer_data[AXIS_Z] *= (float)MAG_Z_SIGN;

	buffer->gyro_data[AXIS_X] *= (float)GYRO_X_SIGN;
	buffer->gyro_data[AXIS_Y] *= (float)GYRO_Y_SIGN;
	buffer->gyro_data[AXIS_Z] *= (float)GYRO_Z_SIGN;
}
