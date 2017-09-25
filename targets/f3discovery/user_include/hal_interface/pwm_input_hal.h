#ifndef PWM_INPUT_HAL
#define PWM_INPUT_HAL	1

#include <stdint.h>

#include "hal_common_includes.h"
#include "stm32f3xx_hal_tim.h"

#define RC_TIMEOUT_THRESHOLD	5U

typedef enum {
	CHANNEL_VALID,
	CHANNEL_INVALID
} rc_channel_validity_enumerator;

typedef struct {
	uint32_t duty_data[5];
	uint32_t period_data[5];
	rc_channel_validity_enumerator channel_states[5];
} rc_input_state;

void pwm_input_hal_init(void);
void get_rc_state(rc_input_state* ret);
void rc_input_watchdog_callback(void);

#endif