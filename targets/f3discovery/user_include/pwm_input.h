#ifndef PWM_INPUT_H
#define PWM_INPUT_H	1

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_common_includes.h"
#include "pwm_input_hal.h"
#include "board_led.h"
#include "sys_timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*
	Requires UART to be set up and stdio redirection functions to be called before UART debug is attempted!!
 */
// #define DEBUG_OVER_UART				1
#define PROVIDE_LED_USER_FEEDBACK 	1

#ifdef DEBUG_OVER_UART
	#include <stdio.h>
#endif

/*
	These values are in us (i.e. microseconds):
 */

#define JS_ENDSTOP_LOW_DEFAULT		1035
#define JS_ENDSTOP_HIGH_DEFAULT		1850

#define RC_LOW_DUTY_NOMINAL_VALUE	986
#define RC_HIGH_DUTY_NOMINAL_VALUE	1972

#define RC_PULSE_MIN_LENGTH			875U
#define RC_PULSE_MAX_LENGTH			2000U

#define MODE5_NORMAL_THRESHOLD		-0.8f
#define MODE5_FAILSAFE_THRESHOLD	0.8f

#define RC_NOMINAL_VALUE_COMPARISON_TOLERANCE	150

#define N_SAMPLES_RC_SIGNAL			200U

#define ROLL_CHANNEL 		4
#define PITCH_CHANNEL 		0
#define YAW_CHANNEL 		3
#define VERTICAL_CHANNEL 	2
#define CHANNEL5_MODE_SWITCH	1

#ifdef PROVIDE_LED_USER_FEEDBACK
	#define ERROR_LED 			LED2
	#define CAL_SUCCESS_LED 	LED1
#endif

#define pwmInputProc_PRIO		3U
#define pwmMonitorProc_PRIO		4U

typedef struct {
	/*
		Data values:
	 */
	float vertical_channel_value;
	float vertical_channel_frequency;
	rc_channel_validity_enumerator vertical_channel_validity;

	float roll_channel_value;
	float roll_channel_frequency;
	rc_channel_validity_enumerator roll_channel_validity;

	float pitch_channel_value;
	float pitch_channel_frequency;
	rc_channel_validity_enumerator pitch_channel_validity;

	float yaw_channel_value;
	float yaw_channel_frequency;
	rc_channel_validity_enumerator yaw_channel_validity;

	float mode_switch_channel_value;
	float mode_switch_channel_frequency;
	rc_channel_validity_enumerator mode_switch_channel_validity;

	/*
		Calibration values:
	 */
	uint32_t vertical_channel_duty_low;
	uint32_t vertical_channel_duty_high;

	uint32_t roll_channel_duty_low;
	uint32_t roll_channel_duty_high;

	uint32_t pitch_channel_duty_low;
	uint32_t pitch_channel_duty_high;

	uint32_t yaw_channel_duty_low;
	uint32_t yaw_channel_duty_high;

	uint32_t mode_switch_channel_duty_low;
	uint32_t mode_switch_channel_duty_high;
} rc_joystick_data_struct;

typedef enum {
	MODE_NORMAL,
	MODE_FAILSAFE,
	VALUE_ERR
} channel5_mode_state;

void setup_pwmInputProcessor(bool calibrate);
bool pwmInputReady(void);
bool pwmInputIsValid(void);
int getPwmInput(rc_joystick_data_struct *js);
int getPwmInputISR(rc_joystick_data_struct *js);
bool pwmInput_ch5FailSafeSet(rc_joystick_data_struct js);

#endif