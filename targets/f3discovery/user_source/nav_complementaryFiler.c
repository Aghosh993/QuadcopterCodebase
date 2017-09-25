#include "nav_complementaryFiler.h"

int compfilterInit(comp_filter_t *c, imu_desc *imuResource, 
					float filter_kp, float filter_ki, float filter_dt)
{
	if(!imuResource)
	{
		return ENULLIMUDESC;
	}
	
	if(!(imuResource->_has_accelerometer))
	{
		return ENOACCEL;
	}

	if(!(imuResource->_has_gyro))
	{
		return ENOGYRO;
	}

	c->sourceImu = imuResource;
	c->_kP = filter_kp;
	c->_kI = filter_ki;
	c->_dt = filter_dt;
	return 0;
}