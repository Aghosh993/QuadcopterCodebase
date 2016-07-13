#ifndef PWM_INPUT_H
#define PWM_INPUT_H	1

#include <stdio.h>
#include <stdint.h>

#include <hal_common_includes.h>
#include <pwm_input_hal.h>
#include <mission_timekeeper.h>

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
#define RC_LOW_DUTY_NOMINAL_VALUE	1100
#define RC_HIGH_DUTY_NOMINAL_VALUE	1800

#define RC_PULSE_MIN_LENGTH			900U
#define RC_PULSE_MAX_LENGTH			2000U

#define RC_NOMINAL_VALUE_COMPARISON_TOLERANCE	100

#define N_SAMPLES_RC_SIGNAL			200U

#define ROLL_CHANNEL 		3
#define PITCH_CHANNEL 		1
#define YAW_CHANNEL 		0
#define VERTICAL_CHANNEL 	2

#ifdef PROVIDE_LED_USER_FEEDBACK
	#define ERROR_LED 			GPIOE, GPIO10
	#define CAL_SUCCESS_LED 	GPIOE, GPIO15
#endif

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
} rc_joystick_data_struct;

void init_rc_inputs(rc_joystick_data_struct* js);
void setup_user_feedback_gpios(void);
void do_rc_channel_callibration(rc_joystick_data_struct* js, uint8_t axis);

void get_rc_input_values(rc_joystick_data_struct* js);

#endif