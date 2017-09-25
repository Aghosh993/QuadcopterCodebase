#include "imu_hal.h"

int imu_hal_init(imu_desc *imu, ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m)
{
	int i = 0;
	float gyroDeadband[3];
	gyroDeadband[0] = 0.0f;
	gyroDeadband[1] = 0.0f;
	gyroDeadband[2] = 0.0f;
	imu_scaled_data_struct imuBuf;

	imu->init();
	
	imu->acc_scale[0] = 1.0f;
	imu->acc_scale[1] = 1.0f;
	imu->acc_scale[2] = 1.0f;

	imu->gyro_scale[0] = 1.0f;
	imu->gyro_scale[1] = 1.0f;
	imu->gyro_scale[2] = 1.0f;

	imu->mag_scale[0] = 1.0f;
	imu->mag_scale[1] = 1.0f;
	imu->mag_scale[2] = 1.0f;

	imu->acc_offset[0] = 0.0f;
	imu->acc_offset[1] = 0.0f;
	imu->acc_offset[2] = 0.0f;

	imu->gyro_offset[0] = 0.0f;
	imu->gyro_offset[1] = 0.0f;
	imu->gyro_offset[2] = 0.0f;

	imu->mag_offset[0] = 0.0f;
	imu->mag_offset[1] = 0.0f;
	imu->mag_offset[2] = 0.0f;

	if(imu->_has_gyro)
	{
		for(i=0; i<100; ++i)
		{
			imu->readAllData(imu);
			gyroDeadband[0] += (float)(imu->gyroRegisterDump[0]);
			gyroDeadband[1] += (float)(imu->gyroRegisterDump[1]);
			gyroDeadband[2] += (float)(imu->gyroRegisterDump[2]);
		}
		imu->gyro_offset[0] = gyroDeadband[0]/100.0f;
		imu->gyro_offset[1] = gyroDeadband[1]/100.0f;
		imu->gyro_offset[2] = gyroDeadband[2]/100.0f;
	}

	if(imu->_has_accelerometer)
	{
		if(imu->setAccScale != NULL)
		{
			if(imu->setAccScale(imu, a) < 0)
			{
				return -1;
			}
		}
		else
		{
			return -2;
		}
	}

	if(imu->_has_gyro)
	{
		if(imu->setGyroScale != NULL)
		{
			if(imu->setGyroScale(imu, g) < 0)
			{
				return -1;
			}
		}
		else
		{
			return -2;
		}
	}

	if(imu->_has_magnetometer)
	{
		if(imu->setMagScale != NULL)
		{
			if(imu->setMagScale(imu, m) < 0)
			{
				return -1;
			}
		}
		else
		{
			return -2;
		}
	}

	return 0;
}

int imu_readScaled(imu_desc *imu, imu_scaled_data_struct *dst)
{
	imu->readAllData(imu);
	if(imu->_has_accelerometer)
	{
		dst->accel_data[0] = ((float)imu->accRegisterDump[0] - imu->acc_offset[0]) * imu->acc_scale[0] * imu->acc_sign[0];
		dst->accel_data[1] = ((float)imu->accRegisterDump[1] - imu->acc_offset[1]) * imu->acc_scale[1] * imu->acc_sign[1];
		dst->accel_data[2] = ((float)imu->accRegisterDump[2] - imu->acc_offset[2]) * imu->acc_scale[2] * imu->acc_sign[2];
	}

	if(imu->_has_gyro)
	{
		dst->gyro_data[0] = ((float)imu->gyroRegisterDump[0] - imu->gyro_offset[0]) * imu->gyro_scale[0] * imu->gyro_sign[0];
		dst->gyro_data[1] = ((float)imu->gyroRegisterDump[1] - imu->gyro_offset[1]) * imu->gyro_scale[1] * imu->gyro_sign[1];
		dst->gyro_data[2] = ((float)imu->gyroRegisterDump[2] - imu->gyro_offset[2]) * imu->gyro_scale[2] * imu->gyro_sign[2];
	}

	if(imu->_has_magnetometer)
	{
		dst->magnetometer_data[0] = ((float)imu->magRegisterDump[0] - imu->mag_offset[0]) * imu->mag_scale[0] * imu->mag_sign[0];
		dst->magnetometer_data[1] = ((float)imu->magRegisterDump[1] - imu->mag_offset[1]) * imu->mag_scale[1] * imu->mag_sign[1];
		dst->magnetometer_data[2] = ((float)imu->magRegisterDump[2] - imu->mag_offset[2]) * imu->mag_scale[2] * imu->mag_sign[2];
	}

	if(imu->_has_temp_sensor)
	{
		
	}
}