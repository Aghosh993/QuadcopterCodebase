#ifndef ALT_CTL_LAW_H_
#define ALT_CTL_LAW_H_	1

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "pwm.h"
#include "pwm_esc.h"
#include "pwm_input.h"
#include "complementary_filter.h"
#include "imu_hal.h"

extern imu_desc mpu9250_spi, l3gd20_spi;

typedef enum {
	MODE_RATE_CONTROL,
	MODE_ATTITUDE_CONTROL
} ctl_mode;

void initAltCtlLaw(void);
void setAltCtrlMode_rateDamper(void);
void setAltCtrlMode_stabilizer(void);

void readIMU(void);
void updateStateVector(void);
void updateUserCmds(rc_joystick_data_struct js);

void calcBodyRateCmds(void);
void bodyRateLoop(void);

float constrainCmd(float var, float min, float max);

#endif