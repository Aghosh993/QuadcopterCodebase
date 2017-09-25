/*
	File: Quadcopter_PWM_HAL.h

	(c) Abhimanyu Ghosh, 2016
 */

#ifndef PWM_ESC_H_
#define PWM_ESC_H_	1

#include <stdint.h>

#include <hal_common_includes.h>
#include "pwm.h"

#define MIN_DUTY 0.5f
#define MAX_DUTY 0.99f

// This configures all vehicle PWM channels:
void esc_init(void);

// Functions to initialize PWM channels corresponding to individual motor/ESC's
void esc1_start(void);
void esc2_start(void);
void esc3_start(void);
void esc4_start(void);

// Functions to set PWM channels corresponding to individual motor/ESC's
// Duty is a value from 0.0 to 1.0 with 1.0 representing full throttle/duty cycle command to the ESC/motor:
void esc1_setDuty(float duty);
void esc2_setDuty(float duty);
void esc3_setDuty(float duty);
void esc4_setDuty(float duty);

#endif