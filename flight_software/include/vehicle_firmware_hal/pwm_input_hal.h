#ifndef PWM_INPUT_HAL
#define PWM_INPUT_HAL 1

#include <stdint.h>

#include <hal_common_includes.h>
#include <interrupts.h>

#define RC_TIMEOUT_THRESHOLD	100U

typedef enum {
	CHANNEL_VALID,
	CHANNEL_INVALID
} rc_channel_validity_enumerator;

typedef struct {
	uint32_t duty_data[4];
	uint32_t period_data[4];
	rc_channel_validity_enumerator channel_states[4];
} rc_input_state;

void pwm_input_init(void);

void setup_timer2_input_capture(void);
void setup_timer3_input_capture(void);
void setup_timer4_input_capture(void);
void setup_timer8_input_capture(void);

void timer2_isr_callback(void);
void timer3_isr_callback(void);
void timer4_isr_callback(void);
void timer8_isr_callback(void);

void rc_input_watchdog_callback(void);

void get_rc_state(rc_input_state* ret);

#endif