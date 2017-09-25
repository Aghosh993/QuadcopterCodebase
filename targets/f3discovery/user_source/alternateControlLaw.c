#include "alternateControlLaw.h"

static volatile complementary_filter_struct socf0;
static volatile imu_scaled_data_struct imuBuf;
static volatile float rollAxisCmd;
static volatile float pitchAxisCmd;
static volatile float yawAxisCmd;
static volatile float throttleCmd;
static volatile float bodyRateCommands[3];
static volatile ctl_mode controlMode;
static volatile bool motorCommandEnable;

void initAltCtlLaw(void)
{
	rollAxisCmd = 0.0f;
	pitchAxisCmd = 0.0f;
	yawAxisCmd = 0.0f;
	throttleCmd = 0.0f;

	bodyRateCommands[0] = 0.0f;
	bodyRateCommands[1] = 0.0f;
	bodyRateCommands[2] = 0.0f;

	controlMode = MODE_RATE_CONTROL;
	motorCommandEnable = false;

	init_complementary_filter(&socf0, &imuBuf, 0.0025f, 0.085f, 1.0f, MODE_2NDORDER_COMPFILTER);
}

void setAltCtrlMode_rateDamper(void)
{
	controlMode = MODE_RATE_CONTROL;
}

void setAltCtrlMode_stabilizer(void)
{
	controlMode = MODE_ATTITUDE_CONTROL;
}

void readIMU(void)
{
	imu_scaled_data_struct accelBuf;
	imu_readScaled(&l3gd20_spi, socf0.imu_data);
	imu_readScaled(&mpu9250_spi, &accelBuf);
	socf0.imu_data->accel_data[0] = accelBuf.accel_data[0];
	socf0.imu_data->accel_data[1] = accelBuf.accel_data[1];
	socf0.imu_data->accel_data[2] = accelBuf.accel_data[2];
}

void updateStateVector(void)
{
	update_complementary_filter(&socf0);
}

void updateUserCmds(rc_joystick_data_struct js)
{
	if(pwmInputIsValid())
	{
		rollAxisCmd = constrainCmd(js.roll_channel_value, -1.0f, 1.0f);
		pitchAxisCmd = constrainCmd(js.pitch_channel_value, -1.0f, 1.0f);
		yawAxisCmd = constrainCmd(js.yaw_channel_value, -1.0f, 1.0f);
		throttleCmd = constrainCmd(js.vertical_channel_value, -1.0f, 1.0f) * 0.5f + 0.5f;
		throttleCmd = constrainCmd(throttleCmd, 0.0f, 0.75f);
		motorCommandEnable = true;			
	}
	else
	{
		rollAxisCmd = 0.0f;
		pitchAxisCmd = 0.0f;
		yawAxisCmd = 0.0f;
		throttleCmd = 0.0f;
		motorCommandEnable = false;
	}
}

#include "sys_timer.h"
extern float vehicleState[3];

void calcBodyRateCmds(void)
{
	if(controlMode == MODE_ATTITUDE_CONTROL)
	{
		updateStateVector();
		vehicleState[0] = sys_timerSeconds();
		vehicleState[1] = socf0.state_vector.roll*180.0f/M_PI;
		vehicleState[2] = socf0.state_vector.pitch*180.0f/M_PI;
		bodyRateCommands[0] = 2.0f*((rollAxisCmd * (float)M_PI/4.0f)-socf0.state_vector.roll);
		bodyRateCommands[1] = 2.0f*((-1.0f * pitchAxisCmd * (float)M_PI/4.0f)-socf0.state_vector.pitch);
		bodyRateCommands[2] = 2.0f*yawAxisCmd*(float)M_PI/4.0f;
	}
	if(controlMode == MODE_RATE_CONTROL)
	{
		bodyRateCommands[0] = 2.2f*((rollAxisCmd * (float)M_PI/4.0f));
		bodyRateCommands[1] = 2.2f*((-1.0f * pitchAxisCmd * (float)M_PI/4.0f));
		bodyRateCommands[2] = 2.2f*yawAxisCmd*(float)M_PI/4.0f;		
	}

}

float constrainCmd(float var, float min, float max)
{
	if(var <= min)
	{
		return min;
	}

	if(var >= max)
	{
		return max;
	}

	return var;
}

void bodyRateLoop(void)
{
	float roll_rate = 	socf0.imu_data->gyro_data[0]*(float)M_PI/180.0f;
	float pitch_rate = 	socf0.imu_data->gyro_data[1]*(float)M_PI/180.0f;
	float yaw_rate = 	socf0.imu_data->gyro_data[2]*(float)M_PI/180.0f;

	float diffX = 0.1f*(bodyRateCommands[0]-roll_rate);
	float diffY = 0.1f*(bodyRateCommands[1]-pitch_rate);
	float diffZ = 0.1f*(bodyRateCommands[2]-yaw_rate);

	float m1Cmd = constrainCmd(throttleCmd + diffX + diffY - diffZ, 0.01f, 0.99f);
	float m2Cmd = constrainCmd(throttleCmd - diffX + diffY + diffZ, 0.01f, 0.99f);
	float m3Cmd = constrainCmd(throttleCmd - diffX - diffY - diffZ, 0.01f, 0.99f);
	float m4Cmd = constrainCmd(throttleCmd + diffX - diffY + diffZ, 0.01f, 0.99f);

	if(motorCommandEnable)
	{
		esc1_setDuty(m1Cmd);
		esc2_setDuty(m2Cmd);
		esc3_setDuty(m3Cmd);
		esc4_setDuty(m4Cmd);
	}
	else
	{
		esc1_setDuty(0.0f);
		esc2_setDuty(0.0f);
		esc3_setDuty(0.0f);
		esc4_setDuty(0.0f);
	}
}