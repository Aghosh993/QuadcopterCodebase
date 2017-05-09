#ifndef PWM_INPUT_H
#define PWM_INPUT_H					1

#include <stdio.h>
#include <stdint.h>

#include <hal_common_includes.h>
#include <pwm_input_hal.h>
#include <mission_timekeeper.h>
#include <rt_telemetry.h>
#include <telem_config.h>

/*
	Requires UART to be set up and uses telemetry libs
 */
#ifdef SEND_PWM_INPUT_TELEMETRY
	#define PWM_INPUT_DEBUG_OVER_UART		1
#endif
#define PROVIDE_LED_USER_FEEDBACK 		1


/*
	These values are in us (i.e. microseconds):
 */
#define RC_LOW_DUTY_NOMINAL_VALUE	815
#define RC_HIGH_DUTY_NOMINAL_VALUE	1450

#define RC_PULSE_MIN_LENGTH			700U
#define RC_PULSE_MAX_LENGTH			1600U

#define MODE5_NORMAL_THRESHOLD		-0.8f
#define MODE5_FAILSAFE_THRESHOLD	0.8f

#define RC_NOMINAL_VALUE_COMPARISON_TOLERANCE	100

#define N_SAMPLES_RC_SIGNAL						200U

#define ROLL_CHANNEL 			4
#define PITCH_CHANNEL 			2
#define YAW_CHANNEL 			1
#define VERTICAL_CHANNEL 		3
#define CHANNEL5_MODE_SWITCH	0

#ifdef PROVIDE_LED_USER_FEEDBACK
	#define ERROR_LED 			LED1
	#define CAL_SUCCESS_LED 	LED2
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

void init_rc_inputs(rc_joystick_data_struct* js, int calibrate);
void do_rc_channel_callibration(rc_joystick_data_struct* js, uint8_t axis);
void get_rc_input_values(rc_joystick_data_struct* js);
channel5_mode_state get_ch5_mode(rc_joystick_data_struct js);

#endif