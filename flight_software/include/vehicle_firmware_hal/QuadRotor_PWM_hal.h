#ifndef QUADROTOR_PWM_HAL_H
#define QUADROTOR_PWM_HAL_H 1

#include <stdint.h>

#include <hal_common_includes.h>

#define MIN_DUTY 0.0f
#define MAX_DUTY 1.0f

#define TIMER_PERIOD_VAL	53333
#define TIMER_PRESCALER_VAL	2

#define TIM_OC_MIN_VAL		22613 //28266 //28270
#define TIM_OC_MAX_VAL		41173 //49599 //49700

// #define TIM_OC_MIN_VAL 		33920
// #define TIM_OC_MAX_VAL		59520

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
void QuadRotor_motor1_setDuty(float duty);
void QuadRotor_motor2_setDuty(float duty);
void QuadRotor_motor3_setDuty(float duty);
void QuadRotor_motor4_setDuty(float duty);

#endif