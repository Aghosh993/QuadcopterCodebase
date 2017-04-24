#ifndef PWM_INPUT_HAL_H_
#define PWM_INPUT_HAL_H_	1

#include "hal_common_includes.h"
#include "board_led.h"

#define RC_LOS_THRESHOLD	2

typedef enum {
	CHANNEL_VALID,
	CHANNEL_INVALID
} rc_channel_validity_enumerator;

typedef struct {
	uint32_t duty_us;
	uint32_t period_us;
} pwm_info_t;

typedef struct {
	uint32_t duty_data[5];
	uint32_t period_data[5];
	rc_channel_validity_enumerator channel_states[5];
} rc_input_state;

void pwm_input_init(void);
void setup_user_feedback_gpios(void);
void get_rc_state(rc_input_state* ret);

void rc_input_validity_watchdog_callback(void);

#endif