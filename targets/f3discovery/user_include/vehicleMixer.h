#ifndef VEHICLE_MIXER_H
#define VEHICLE_MIXER_H		1

/*
	Standard includes:
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*
	FreeRTOS headers:
 */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*
	BSP Headers:
 */
#include "pwm_esc.h"

#define TORQUE_Kx	1.0f
#define TORQUE_Ky	1.0f
#define TORQUE_Kz	1.0f

#define THROTTLE_LIMIT 0.75f

typedef struct {
	float motor1_cmd;
	float motor2_cmd;
	float motor3_cmd;
	float motor4_cmd;
} mixerActuatorCmds_t;

void mixerInit(void);
void mixerCalActuators(void);
void mixerExertTorques(float Tx, float Ty, float Tz);
void mixerSetThrust(float thr);
void mixerISRCallback(pwm_channel ch);
bool mixerIsReady(void);

#endif