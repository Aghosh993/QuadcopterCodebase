/*
	File: Quadcopter_PWM_HAL.h

	(c) Abhimanyu Ghosh, 2016
 */

#ifndef QUADCOPTER_PWM_HAL_H_
#define QUADCOPTER_PWM_HAL_H_	1

#include <stdint.h>

#include <hal_common_includes.h>

#define MIN_DUTY 0.4f//0.35f
#define MAX_DUTY 0.8f//0.85f

// For the old hardware  platform (With rev 1 interface board):
// #define MOTOR4_HET_CHANNEL	hetRAM1, pwm0
// #define MOTOR1_HET_CHANNEL	hetRAM1, pwm1
// #define MOTOR2_HET_CHANNEL	hetRAM1, pwm2
// #define MOTOR3_HET_CHANNEL	hetRAM1, pwm3

// For new hardware platform (With rev 2 interface board containing power drivers and BNO055):
#define MOTOR2_HET_CHANNEL	hetRAM1, pwm0
#define MOTOR3_HET_CHANNEL	hetRAM1, pwm1
#define MOTOR4_HET_CHANNEL	hetRAM1, pwm2
#define MOTOR1_HET_CHANNEL	hetRAM1, pwm3

// This configures all vehicle PWM channels:
void QuadRotor_PWM_init(void);

// Functions to initialize PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_start(void);
void QuadRotor_motor2_start(void);
void QuadRotor_motor3_start(void);
void QuadRotor_motor4_start(void);

// Functions to stop PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_stop(void);
void QuadRotor_motor2_stop(void);
void QuadRotor_motor3_stop(void);
void QuadRotor_motor4_stop(void);

// Functions to set PWM channels corresponding to individual motor/ESC's
// Duty is a value from 0.0 to 1.0 with 1.0 representing full throttle/duty cycle command to the ESC/motor:
void QuadRotor_motor1_setDuty(float duty);
void QuadRotor_motor2_setDuty(float duty);
void QuadRotor_motor3_setDuty(float duty);
void QuadRotor_motor4_setDuty(float duty);

void QuadRotor_motor1_setDuty_raw(float duty);
void QuadRotor_motor2_setDuty_raw(float duty);
void QuadRotor_motor3_setDuty_raw(float duty);
void QuadRotor_motor4_setDuty_raw(float duty);

int is_setup(int motor);

#endif