#ifndef PWM_H_
#define PWM_H_	1

#include <stdint.h>
#include <stdbool.h>

#include "hal_common_includes.h"

#define TIMER_PERIOD_400HZ	60000
#define TIMER_PERIOD_500HZ	48000

typedef enum {
	pwm1,
	pwm2,
	pwm3,
	pwm4
} pwm_channel;

void init_pwm(void);
void set_pwm(pwm_channel ch, float duty);
bool pwmReadyForNewCmd(void);
void disablePwmIrq(void);
void enablePwmIrq(void);

#endif