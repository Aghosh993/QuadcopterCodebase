#include "vehicleMixer.h"
#include "cpu_hal_interface.h"

volatile QueueHandle_t actuatorCmdQueue;
static volatile float lastThrustCmd, lastDiffX, lastDiffY, lastDiffZ;
static volatile bool mixerReady;

/*
	Initialize vehicle mixer by calling BSP PWM init after initializing data structs.
	MUST be called from userspace context!!
 */
void mixerInit(void)
{
	mixerReady = false;
	lastDiffX = 0.0f;
	lastDiffY = 0.0f;
	lastDiffZ = 0.0f;
	lastThrustCmd = 0.0f;

	actuatorCmdQueue = xQueueCreate((UBaseType_t)1, sizeof(mixerActuatorCmds_t));
	if(actuatorCmdQueue == 0)
	{
		while(1);
	}
	esc_init();
	mixerReady = true;
}

void mixerCalActuators(void)
{
	mixerReady = false;
	float duty = 0.8f;
	set_pwm(pwm1, duty);
	set_pwm(pwm2, duty);
	set_pwm(pwm3, duty);
	set_pwm(pwm4, duty);
	cpu_sw_delay(100);
	while(duty > 0.4f)
	{
		// esc1_setDuty(duty);
		// esc2_setDuty(duty);
		// esc3_setDuty(duty);
		// esc4_setDuty(duty);
		set_pwm(pwm1, duty);
		set_pwm(pwm2, duty);
		set_pwm(pwm3, duty);
		set_pwm(pwm4, duty);
		cpu_sw_delay(3);
		duty -= 0.01f;
	}
	cpu_sw_delay(100);
	mixerReady = true;
}

/*
	Apply body-fixed torques to the vehicle by creating and sending necessary 
	actuator commands to the actuation interrupt process.
	Tx, Ty, Tz are body-fixed normalized torques, and are thus unit-less and 
	must be in the range [-1,1]

	MUST be called in the userspace context!!
*/
void mixerExertTorques(float Tx, float Ty, float Tz)
{
	if(Tx >= 1.0f)
	{
		lastDiffX = TORQUE_Kx;
	}
	else
	{
		if(Tx <= -1.0f)
		{
			lastDiffX = -1.0f*TORQUE_Kx;
		}
		else
		{
			lastDiffX = Tx*TORQUE_Kx;
		}
	}

	if(Ty >= 1.0f)
	{
		lastDiffY = TORQUE_Ky;
	}
	else
	{
		if(Ty <= -1.0f)
		{
			lastDiffY = -1.0f*TORQUE_Ky;
		}
		else
		{
			lastDiffY = Ty*TORQUE_Ky;
		}
	}

	if(Tz >= 1.0f)
	{
		lastDiffZ = TORQUE_Kz;
	}
	else
	{
		if(Tz <= -1.0f)
		{
			lastDiffZ = -1.0f*TORQUE_Kz;
		}
		else
		{
			lastDiffZ = Tz*TORQUE_Kz;
		}
	}

	mixerActuatorCmds_t actCmd;
	actCmd.motor1_cmd = lastThrustCmd + lastDiffX*0.5f + lastDiffY*0.5f - lastDiffZ;
	actCmd.motor2_cmd = lastThrustCmd - lastDiffX*0.5f + lastDiffY*0.5f + lastDiffZ;
	actCmd.motor3_cmd = lastThrustCmd - lastDiffX*0.5f - lastDiffY*0.5f - lastDiffZ;
	actCmd.motor4_cmd = lastThrustCmd + lastDiffX*0.5f - lastDiffY*0.5f + lastDiffZ;
	xQueueOverwrite(actuatorCmdQueue, &actCmd);
}

/*
	Apply body-fixed thrust as a common collective value to all actuators.
	Acceptable range is [0, THROTTLE_LIMIT] where THROTTLE_LIMIT is a parameter
	between 0 and 1, configured in vehicleMixer.h to prevent actuator saturation.

	MUST be called in the userspace context!!
 */
void mixerSetThrust(float thr)
{
	if(thr >= 0.0f && thr < THROTTLE_LIMIT)
	{
		lastThrustCmd = thr;	
	}
	else
	{
		if(thr < 0.0f)
		{
			lastThrustCmd = 0.0f;
		}
		if(thr > THROTTLE_LIMIT)
		{
			lastThrustCmd = THROTTLE_LIMIT;
		}
	}

	mixerActuatorCmds_t actCmd;
	actCmd.motor1_cmd = lastThrustCmd + lastDiffX + lastDiffY - lastDiffZ;
	actCmd.motor2_cmd = lastThrustCmd - lastDiffX + lastDiffY + lastDiffZ;
	actCmd.motor3_cmd = lastThrustCmd - lastDiffX - lastDiffY - lastDiffZ;
	actCmd.motor4_cmd = lastThrustCmd + lastDiffX - lastDiffY + lastDiffZ;

	if(actCmd.motor1_cmd > 1.0f)
	{
		actCmd.motor1_cmd = 1.0f;
	}
	if(actCmd.motor1_cmd < 0.0f)
	{
		actCmd.motor1_cmd = 0.0f;
	}

	if(actCmd.motor2_cmd > 1.0f)
	{
		actCmd.motor2_cmd = 1.0f;
	}
	if(actCmd.motor2_cmd < 0.0f)
	{
		actCmd.motor2_cmd = 0.0f;
	}

	if(actCmd.motor3_cmd > 1.0f)
	{
		actCmd.motor3_cmd = 1.0f;
	}
	if(actCmd.motor3_cmd < 0.0f)
	{
		actCmd.motor3_cmd = 0.0f;
	}

	if(actCmd.motor4_cmd > 1.0f)
	{
		actCmd.motor4_cmd = 1.0f;
	}
	if(actCmd.motor4_cmd < 0.0f)
	{
		actCmd.motor4_cmd = 0.0f;
	}

	xQueueOverwrite(actuatorCmdQueue, &actCmd);	
}

void mixerISRCallback(pwm_channel ch)
{
	// mixerActuatorCmds_t cmds;
	/*
		Only set PWM outputs if we've written at least 1 initial command into
		the actuation command queue. This gates the code below from running prior
		to full PWM setup and ESC calibration sequences in mixerInit().
	 */
	// if(uxQueueMessagesWaitingFromISR(actuatorCmdQueue) > 0)
	// {
	// 	xQueuePeekFromISR(actuatorCmdQueue, &cmds);
	// 	switch(ch)
	// 	{
	// 		case pwm1:
	// 			esc1_setDuty(cmds.motor1_cmd);
	// 			break;
	// 		case pwm2:
	// 			esc2_setDuty(cmds.motor2_cmd);
	// 			break;
	// 		case pwm3:
	// 			esc3_setDuty(cmds.motor3_cmd);
	// 			break;
	// 		case pwm4:
	// 			esc4_setDuty(cmds.motor4_cmd);
	// 			break;
	// 		default:
	// 			break;
	// 	}
	// }	
}

bool mixerIsReady(void)
{
	return mixerReady;
}