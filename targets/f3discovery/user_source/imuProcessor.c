#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "imuProcessor.h"
#include "board_button.h"
// #include "board_led.h"
#include "quaternion.h"
#include "attitudeControl.h"
#include "vehicleMixer.h"
#include "pwm_input.h"
#include "complementary_filter.h"

extern imu_desc mpu9250_spi, lsm303_i2c;

float vehicleRoll, vehiclePitch, vehicleYaw;

void basicStabilizerTask(void *pvParameters)
{
	vTaskDelay(pdMS_TO_TICKS(500));
	board_button_init();
	
	vTaskDelay(pdMS_TO_TICKS(500));

	imu_scaled_data_struct imuBuf;
	int i = 0;
	float gyroDeadband[3];
	for(i=0; i<100; ++i)
	{
		imu_readScaled(&mpu9250_spi, &imuBuf);
		gyroDeadband[0] += imuBuf.gyro_data[0];
		gyroDeadband[1] += imuBuf.gyro_data[1];
		gyroDeadband[2] += imuBuf.gyro_data[2];
	}
	gyroDeadband[0] /= 100.0f;
	gyroDeadband[1] /= 100.0f;
	gyroDeadband[2] /= 100.0f;

	float roll_rate_cmd = 0.0f;
	float pitch_rate_cmd = 0.0f;
	float yaw_rate_cmd = 0.0f;

	float roll = 0.0f;
	float pitch = 0.0f;

	rc_joystick_data_struct rcData;

	float roll_rate, pitch_rate, yaw_rate;
	roll_rate = 0.0f;
	pitch_rate = 0.0f;
	yaw_rate = 0.0f;

	complementary_filter_struct socf0;
	init_complementary_filter(&socf0, &imuBuf, 0.002f, 0.085f, 1.0f, MODE_2NDORDER_COMPFILTER);
	float throttleVal = 0.0f;
	float diffX = 0.0f;
	float diffY = 0.0f;
	float diffZ = 0.0f;

	while(1)
	{
		board_led_on(LED2);
		imu_readScaled(&mpu9250_spi, &imuBuf);
		board_led_off(LED2);
		update_complementary_filter(&socf0);

		roll_rate = (imuBuf.gyro_data[0]-gyroDeadband[0])*(float)M_PI/180.0f;
		pitch_rate = (imuBuf.gyro_data[1]-gyroDeadband[1])*(float)M_PI/180.0f;
		yaw_rate = (imuBuf.gyro_data[2]-gyroDeadband[2])*(float)M_PI/180.0f;

		// roll = 0.98f*(roll + roll_rate*0.002f) + 0.02f*(imuBuf.accel_data[1]/9.8f);
		// pitch = 0.98f*(pitch + pitch_rate*0.002f) + 0.02f*(-1.0f*imuBuf.accel_data[0]/9.8f);
		
		roll = socf0.state_vector.roll;
		pitch = socf0.state_vector.pitch;
		
		vehicleRoll = socf0.state_vector.roll * 180.0f/M_PI;
		vehiclePitch = socf0.state_vector.pitch * 180.0f/M_PI;

		// vehicleRoll = roll * 180.0f/M_PI;
		// vehiclePitch = pitch * 180.0f/M_PI;

		if(getPwmInput(&rcData) == 0)
		{
			if(pwmInputIsValid())
			{
				roll_rate_cmd = 1.4f*((rcData.roll_channel_value * (float)M_PI/4.0f)-roll);
				pitch_rate_cmd = 1.4f*((-1.0f * rcData.pitch_channel_value * (float)M_PI/4.0f)-pitch);
				yaw_rate_cmd = 1.4f*rcData.yaw_channel_value*(float)M_PI/4.0f;				
				throttleVal = (rcData.vertical_channel_value*0.5f) + 0.5f;
			}
			// LOS:
			else
			{
				// mixerSetThrust(0.0f);
				// mixerExertTorques(0.0f, 0.0f, 0.0f);
				esc1_setDuty(0.0f);
				esc2_setDuty(0.0f);
				esc3_setDuty(0.0f);
				esc4_setDuty(0.0f);
			}
		}

		// mixerExertTorques(0.1f*(roll_rate_cmd-roll_rate),
		// 					0.1f*(pitch_rate_cmd-pitch_rate),
		// 					0.2f*(yaw_rate_cmd-yaw_rate));
		diffX = 0.1f*(roll_rate_cmd-roll_rate);
		diffY = 0.1f*(pitch_rate_cmd-pitch_rate);
		diffZ = 0.2f*(yaw_rate_cmd-yaw_rate);
		esc1_setDuty(throttleVal + diffX + diffY - diffZ);
		esc2_setDuty(throttleVal - diffX + diffY + diffZ);
		esc3_setDuty(throttleVal - diffX - diffY - diffZ);
		esc4_setDuty(throttleVal + diffX - diffY + diffZ);

		vTaskDelay(pdMS_TO_TICKS(4));
	}
}

void setup_imuProcessor(void)
{
	// xTaskCreate(
		// imuProcessorTask,                    	/* Function pointer */
		// "imuProcessorTask",                  	/* Task name - for debugging only*/
		// configMINIMAL_STACK_SIZE,         	/* Stack depth in words */
		// (void*) NULL,                     	/* Pointer to tasks arguments (parameter) */
		// tskIDLE_PRIORITY + imuProc_PRIO,      /* Task priority*/
		// NULL                              	/* Task handle */
	// );
	xTaskCreate(
		basicStabilizerTask,                    	/* Function pointer */
		"basicStabilizerTask",                  	/* Task name - for debugging only*/
		configMINIMAL_STACK_SIZE,         	/* Stack depth in words */
		(void*) NULL,                     	/* Pointer to tasks arguments (parameter) */
		tskIDLE_PRIORITY + imuProc_PRIO,      /* Task priority*/
		NULL                              	/* Task handle */
	);
}

// void innerLoopUpdateTask(void *pvParameters)
// {

// }

void imuProcessorTask(void *pvParameters)
{
	vTaskDelay(pdMS_TO_TICKS(500));
	board_button_init();
	int ret = imu_hal_init(&mpu9250_spi, SCALE_2G, SCALE_1000_DPS, SCALE_1POINT3_GAUSS);
	if(ret<0)
	{
		printf("ERROR in IMU1 init, errcode %d!!\r\n", ret);
		while(1);
	}

	ret = imu_hal_init(&lsm303_i2c, SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS);
	if(ret<0)
	{
		printf("ERROR in IMU2 init, errcode %d!!\r\n", ret);
		while(1);
	}
	vTaskDelay(pdMS_TO_TICKS(500));

	float heading = 0.0f;

	imu_scaled_data_struct imuBuf, imu2Buf;

	int i = 0;
	float gyroDeadband[3];
	for(i=0; i<100; ++i)
	{
		imu_readScaled(&mpu9250_spi, &imu2Buf);
		gyroDeadband[0] += imu2Buf.gyro_data[0];
		gyroDeadband[1] += imu2Buf.gyro_data[1];
		gyroDeadband[2] += imu2Buf.gyro_data[2];
	}
	gyroDeadband[0] /= 100.0f;
	gyroDeadband[1] /= 100.0f;
	gyroDeadband[2] /= 100.0f;

	float delT = (float)imuProc_pdMS/(float)1000.0f;
	
	vector3d_t imuRates;
	vector3d_t accelData, magData;
	vector3d_t vert_vect = {0.0f, 0.0f, 1.0f};
	// vector3d_t localDeclination = {cosf(12.1f*M_PI/180.0f), sinf(12.1f*M_PI/180.0f), 0.0f};

	quaternion_t setpoint = {0.0f, 0.0f, 0.0f, 1.0f};

	// while(1)
	// {
	// 	while(!board_button_read(BTN_USER));
	// 	imu_readScaled(&lsm303_i2c, &imuBuf);
	// 	magData.i = imuBuf.magnetometer_data[0];
	// 	magData.j = imuBuf.magnetometer_data[1];
	// 	magData.k = imuBuf.magnetometer_data[2];
	// 	printf("%f, %f, %f\r\n", magData.i, magData.j, magData.k);
	// 	vTaskDelay(pdMS_TO_TICKS(250));
	// }

	imu_readScaled(&lsm303_i2c, &imuBuf);
	magData.i = imuBuf.magnetometer_data[0];
	magData.j = imuBuf.magnetometer_data[1];
	magData.k = imuBuf.magnetometer_data[2];
	scaleVect3d(magData,1.0f/magVect3d(magData));
	vector3d_t localDeclination = {magData.i, magData.j, 0.0f};
	vector3d_t attitudeInitial = {0.0f, 0.0f, 0.0f};
	quaternion_t att = vector3d_toQuaternion(attitudeInitial);
	quaternion_normalize(&att);
	vector3d_t att_euler = quaternion_toEuler(att);

	attitudeController_t vehicleCtrl;
	attitudeControllerInit(&vehicleCtrl, 20.0f, 0.0f);

	rc_joystick_data_struct rcData;

	mixerSetThrust(0.0f);
	mixerExertTorques(0.0f, 0.0f, 0.0f);
	vector3d_t attitude_cmd;
	attitude_cmd.i = 0.0f;
	attitude_cmd.j = 0.0f;
	attitude_cmd.k = 0.0f;
	vector3d_t cmds;

	while(1)
	{
		if(getPwmInput(&rcData) == 0)
		{
			if(pwmInputIsValid())
			{
				mixerSetThrust((rcData.vertical_channel_value*0.5f) + 0.5f);
				mixerExertTorques(-1.0f*cmds.i, -1.0f*cmds.j, -1.0f*cmds.k);
				attitude_cmd.i = rcData.roll_channel_value * (float)M_PI/4.0f;
				attitude_cmd.j = -1.0f*rcData.pitch_channel_value * (float)M_PI/4.0f;
				if(fabs(rcData.yaw_channel_value) > 0.05f)
				{
					attitude_cmd.k += rcData.yaw_channel_value * 0.01f;
				}
			}
			// LOS:
			else
			{
				mixerSetThrust(0.0f);
				mixerExertTorques(0.0f, 0.0f, 0.0f);
			}
		}
		/*
			Get IMU data over I2C and SPI bus.
			Benchmarked at around 600 Microseconds using 400 kHz I2C 
			and 1 Mbit/s SPI clock, and 72 MHz system clock on an
			STM32F303VCT6 chip
		 */
		imu_readScaled(&mpu9250_spi, &imu2Buf);
		imu_readScaled(&lsm303_i2c, &imuBuf);
		
		/*
			Actual sensor fusion process below.
			Benchmarked at around 33 Microseconds on an STM32F303VCT6
			running at system clock of 72 MHz
		 */
		imuRates.i = (imu2Buf.gyro_data[0] - gyroDeadband[0]) * M_PI/180.0f;
		imuRates.j = (imu2Buf.gyro_data[1] - gyroDeadband[1]) * M_PI/180.0f;
		imuRates.k = (imu2Buf.gyro_data[2] - gyroDeadband[2]) * M_PI/180.0f;

		accelData.i = imu2Buf.accel_data[0] / 9.810f;
		accelData.j = imu2Buf.accel_data[1] / 9.810f;
		accelData.k = imu2Buf.accel_data[2] / 9.810f;

		magData.i = imuBuf.magnetometer_data[0];
		magData.j = imuBuf.magnetometer_data[1];
		magData.k = imuBuf.magnetometer_data[2];

		/*
			Rotate accelerometer and magnetometer body-fixed measurements into
			world frame, and zero out world-vertical component of magnetometer
			measurement as we aren't handling inclination here:
		 */
		vector3d_t accel_world = quaternion_rotateVect3d(accelData, att);
		vector3d_t mag_world = quaternion_rotateVect3d(magData, att);
		mag_world.k = 0.0f;

		/*
			Normalize magnetic world vector to unit vector:
		 */
		scaleVect3d(magData,1.0f/magVect3d(magData));

		/*
			Compute corrections for gyro using the accelerometer and magnetometer data,
			in the world frame. These corrections are essentially rate trims:
		 */
		// vector3d_t gyroCorrection = addVect3d(crossProd(accel_world, vert_vect), 
		// 										crossProd(mag_world, localDeclination));
		vector3d_t gyroCorrection = crossProd(accel_world, vert_vect);

		/*
			Rotate the corrections into the body frame using an inverse rotation
			using the current attitude:
		 */
		vector3d_t gyroCorrection_body = quaternion_invRotateVect3d(gyroCorrection, att);

		/*
			Add the correction in body frame to the strapdown gyro 
			rates in the body frame, and then convert to a rotation quaternion
			using axis-angle representation:
		 */
		imuRates = addVect3d(imuRates, scaleVect3d(gyroCorrection_body, 0.1f));
		quaternion_t gyroDelTheta = quaternion_fromRotation(magVect3d(imuRates)*delT, scaleVect3d(imuRates,1.0f/magVect3d(imuRates)));
		
		/*
			Normalize the new dTheta to be a proper rotation quaternion:
		 */
		quaternion_normalize(&gyroDelTheta);

		/*
			Now, actually perform the integration of latest dTheta and 
			update attitude quaternion:
		 */
		att = quaternion_mul(att, gyroDelTheta);
		quaternion_normalize(&att);

		/*
			Perform attitude control using latest computed attitude quaternion:
		 */
		vector3d_t adj = attitudeController_computeTorques(vehicleCtrl, 
															vector3d_toQuaternion(attitudeInitial),
															att,
															imuRates);

		att_euler = quaternion_toEuler(att);
		vehicleRoll = att_euler.i;
		vehiclePitch = att_euler.j;
		vehicleYaw = att_euler.k;

		cmds = attitudeController_computeTorques(vehicleCtrl, 
															vector3d_toQuaternion(attitude_cmd), 
															att, imuRates);
		// printf("%f %f %f\r\n", cmds.i, cmds.j, cmds.k);
		// vehicleRoll = cmds.i;
		// vehiclePitch = cmds.j;
		// vehicleYaw = cmds.k;

		vTaskDelay(pdMS_TO_TICKS(imuProc_pdMS));
	}
}