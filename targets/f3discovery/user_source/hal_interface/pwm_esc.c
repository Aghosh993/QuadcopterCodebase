#include "pwm_esc.h"

// This configures all vehicle PWM channels:
void esc_init(void)
{
	init_pwm();
}

// Functions to initialize PWM channels corresponding to individual motor/ESC's
void esc1_start(void)
{
	set_pwm(pwm1, MIN_DUTY);
}

void esc2_start(void)
{
	set_pwm(pwm2, MIN_DUTY);
}

void esc3_start(void)
{
	set_pwm(pwm3, MIN_DUTY);
}

void esc4_start(void)
{
	set_pwm(pwm4, MIN_DUTY);
}

// Functions to set PWM channels corresponding to individual motor/ESC's
// Duty is a value from 0.0 to 1.0 with 1.0 representing full throttle/duty cycle command to the ESC/motor:
void esc1_setDuty(float duty)
{
	float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	set_pwm(pwm1, duty_scaled);
}

void esc2_setDuty(float duty)
{
	float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	set_pwm(pwm2, duty_scaled);
}

void esc3_setDuty(float duty)
{
	float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	set_pwm(pwm3, duty_scaled);
}

void esc4_setDuty(float duty)
{
	float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	set_pwm(pwm4, duty_scaled);
}