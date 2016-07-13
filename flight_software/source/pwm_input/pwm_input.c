#include <pwm_input.h>

static uint8_t rc_duty_isOutOfBounds(uint32_t pulseLength)
{
	if(pulseLength < RC_PULSE_MIN_LENGTH || pulseLength > RC_PULSE_MAX_LENGTH)
	{
		return 1U;
	}
	return 0U;
}

static uint8_t rc_duty_isNearNominalLimit(uint32_t pulseLength, uint32_t nominal_length)
{
	int32_t diff = (int32_t)pulseLength - (int32_t)nominal_length;
	/*
		Basically, an absolute-value comparison to determine if signal is 
		within bounds of expected quantity:
	 */
	if(((diff >= 0) && diff < RC_NOMINAL_VALUE_COMPARISON_TOLERANCE) ||
		((diff < 0) && diff > -1 * (int32_t)RC_NOMINAL_VALUE_COMPARISON_TOLERANCE))
	{
		return 1;
	}
	return 0;
}

void init_rc_inputs(rc_joystick_data_struct* js)
{
	pwm_input_init();
	#ifdef PROVIDE_LED_USER_FEEDBACK
		setup_user_feedback_gpios();
	#endif
	do_rc_channel_callibration(js, ROLL_CHANNEL);
	do_rc_channel_callibration(js, PITCH_CHANNEL);
	do_rc_channel_callibration(js, YAW_CHANNEL);
	do_rc_channel_callibration(js, VERTICAL_CHANNEL);

	js->roll_channel_value = 0.0f;
	js->pitch_channel_value = 0.0f;
	js->yaw_channel_value = 0.0f;
	js->vertical_channel_value = -1.0f;
}

void do_rc_channel_callibration(rc_joystick_data_struct* js, uint8_t axis)
{
	uint8_t samples_iterator = 0U;
	uint32_t averaging_sum = 0U;
	uint8_t i = 0U;

	#ifdef DEBUG_OVER_UART
		printf("Starting RC calibration procedure on axis %d:\r\n", axis);
	#endif
	rc_input_state st;
	get_rc_state(&st);
	/*
		Block until all RC inputs are sane (i.e. within expected bounds for RC pulse lengths):
	 */
	gpio_clear(CAL_SUCCESS_LED);
	gpio_set(ERROR_LED);

	while(rc_duty_isOutOfBounds(st.duty_data[axis]))
	{
		get_rc_state(&st);
	}
	#ifdef DEBUG_OVER_UART
		printf("RC Input sane on this channel, proceeding\r\n");
	#endif
	gpio_clear(ERROR_LED);

	/*
		Low limit calibration:
	 */

	#ifdef DEBUG_OVER_UART
	switch(axis)
	{
		case ROLL_CHANNEL:
			printf("Calibrating roll stick low. Please move the right stick x-axis to the left limit\r\n");
			break;
		case PITCH_CHANNEL:
			printf("Calibrating pitch stick low. Please move the right stick y-axis to the lower limit\r\n");
			break;
		case YAW_CHANNEL:
			printf("Calibrating yaw stick low. Please move the left stick x-axis to the left limit\r\n");
			break;
		case VERTICAL_CHANNEL:
			printf("Calibrating vertical stick low. Please move the left stick y-axis to the lower limit\r\n");
			break;
	}
	#endif
	/*
		Wait for user to move yaw stick to left limit:
	 */
	gpio_set(ERROR_LED);
	while(!rc_duty_isNearNominalLimit(st.duty_data[axis], RC_LOW_DUTY_NOMINAL_VALUE))
	{
		get_rc_state(&st);
	}
	gpio_clear(ERROR_LED);
	/* 	
	 	Wait 500 milliseconds for input to stabilize 
	 	and user to reach stick end stop:
	 */
	timekeeper_delay(500U);

	#ifdef DEBUG_OVER_UART
		printf("Obtaining samples now. Please hold stick at limit\r\n");
	#endif

	/*
		Take N samples (settable in header file) and average:
	 */
	for(samples_iterator = 0U; samples_iterator < N_SAMPLES_RC_SIGNAL; ++samples_iterator)
	{
		get_rc_state(&st);
		averaging_sum += st.duty_data[axis];
	}
	switch(axis)
	{
		case ROLL_CHANNEL:
			js->roll_channel_duty_low = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
		case PITCH_CHANNEL:
			js->pitch_channel_duty_low = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
		case YAW_CHANNEL:
			js->yaw_channel_duty_low = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
		case VERTICAL_CHANNEL:
			js->vertical_channel_duty_low = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
	}

	/*
		Success!
	 */
	#ifdef DEBUG_OVER_UART
		switch(axis)
		{
			case ROLL_CHANNEL:
				printf("Roll axis low successfully calibrated! Low endstop set to %d\r\n", js->roll_channel_duty_low);
				break;
			case PITCH_CHANNEL:
				printf("Pitch axis low successfully calibrated! Low endstop set to %d\r\n", js->pitch_channel_duty_low);
				break;
			case YAW_CHANNEL:
				printf("Yaw axis low successfully calibrated! Low endstop set to %d\r\n", js->yaw_channel_duty_low);
				break;
			case VERTICAL_CHANNEL:
				printf("Vertical axis low successfully calibrated! Low endstop set to %d\r\n", js->vertical_channel_duty_low);
				break;
		}
	#endif
	for(i = 0U; i < 2U; ++i)
	{
		gpio_set(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
		gpio_clear(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
	}

	averaging_sum = 0U;

	/*
		High limit calibration:
	 */

	#ifdef DEBUG_OVER_UART
	switch(axis)
	{
		case ROLL_CHANNEL:
			printf("Calibrating roll stick high. Please move the right stick x-axis to the right limit\r\n");
			break;
		case PITCH_CHANNEL:
			printf("Calibrating pitch stick high. Please move the right stick y-axis to the upper limit\r\n");
			break;
		case YAW_CHANNEL:
			printf("Calibrating yaw stick high. Please move the left stick x-axis to the right limit\r\n");
			break;
		case VERTICAL_CHANNEL:
			printf("Calibrating vertical stick high. Please move the left stick y-axis to the upper limit\r\n");
			break;
	}
	#endif
	/*
		Wait for user to move yaw stick to left limit:
	 */
	gpio_set(ERROR_LED);
	while(!rc_duty_isNearNominalLimit(st.duty_data[axis], RC_HIGH_DUTY_NOMINAL_VALUE))
	{
		get_rc_state(&st);
	}
	gpio_clear(ERROR_LED);
	/* 	
	 	Wait 500 milliseconds for input to stabilize 
	 	and user to reach stick end stop:
	 */
	timekeeper_delay(500U);

	#ifdef DEBUG_OVER_UART
		printf("Obtaining samples now. Please hold stick at limit\r\n");
	#endif

	/*
		Take N samples (settable in header file) and average:
	 */
	for(samples_iterator = 0U; samples_iterator < N_SAMPLES_RC_SIGNAL; ++samples_iterator)
	{
		get_rc_state(&st);
		averaging_sum += st.duty_data[axis];
	}
	switch(axis)
	{
		case ROLL_CHANNEL:
			js->roll_channel_duty_high = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
		case PITCH_CHANNEL:
			js->pitch_channel_duty_high = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
		case YAW_CHANNEL:
			js->yaw_channel_duty_high = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
		case VERTICAL_CHANNEL:
			js->vertical_channel_duty_high = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
	}

	/*
		Success!
	 */
	#ifdef DEBUG_OVER_UART
		switch(axis)
		{
			case ROLL_CHANNEL:
				printf("Roll axis high successfully calibrated! High endstop set to %d\r\n", js->roll_channel_duty_high);
				break;
			case PITCH_CHANNEL:
				printf("Pitch axis high successfully calibrated! High endstop set to %d\r\n", js->pitch_channel_duty_high);
				break;
			case YAW_CHANNEL:
				printf("Yaw axis high successfully calibrated! High endstop set to %d\r\n", js->yaw_channel_duty_high);
				break;
			case VERTICAL_CHANNEL:
				printf("Vertical axis high successfully calibrated! High endstop set to %d\r\n", js->vertical_channel_duty_high);
				break;
		}
	#endif
	for(i = 0U; i < 2U; ++i)
	{
		gpio_set(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
		gpio_clear(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
	}
}

void setup_user_feedback_gpios(void)
{
	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Set GPIO9 and 11 (in GPIO port E) to 'output push-pull'. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10 | GPIO15);
}

void get_rc_input_values(rc_joystick_data_struct* js)
{
	uint32_t roll_channel_limits_range = js->roll_channel_duty_high-js->roll_channel_duty_low;
	uint32_t pitch_channel_limits_range = js->pitch_channel_duty_high-js->pitch_channel_duty_low;
	uint32_t yaw_channel_limits_range = js->yaw_channel_duty_high-js->yaw_channel_duty_low;
	uint32_t vertical_channel_limits_range = js->vertical_channel_duty_high-js->vertical_channel_duty_low;

	uint32_t roll_channel_limits_sum = js->roll_channel_duty_high+js->roll_channel_duty_low;
	uint32_t pitch_channel_limits_sum = js->pitch_channel_duty_high+js->pitch_channel_duty_low;
	uint32_t yaw_channel_limits_sum = js->yaw_channel_duty_high+js->yaw_channel_duty_low;
	uint32_t vertical_channel_limits_sum = js->vertical_channel_duty_high+js->vertical_channel_duty_low;

	rc_input_state st;
	get_rc_state(&st);
	
	if(st.duty_data[ROLL_CHANNEL] < js->roll_channel_duty_low)
	{
		js->roll_channel_value = -1.0f;
	}
	if(st.duty_data[ROLL_CHANNEL] > js->roll_channel_duty_high)
	{
		js->roll_channel_value = 1.0f;
	}
	else
	{
		js->roll_channel_value = ((float)2.0 * (float)st.duty_data[ROLL_CHANNEL] - (float)roll_channel_limits_sum)/(float)roll_channel_limits_range;
	}

	if(st.duty_data[PITCH_CHANNEL] < js->pitch_channel_duty_low)
	{
		js->pitch_channel_value = -1.0f;
	}
	if(st.duty_data[PITCH_CHANNEL] > js->pitch_channel_duty_high)
	{
		js->pitch_channel_value = 1.0f;
	}
	else
	{
		js->pitch_channel_value = ((float)2.0 * (float)st.duty_data[PITCH_CHANNEL] - (float)pitch_channel_limits_sum)/(float)pitch_channel_limits_range;
	}

	if(st.duty_data[YAW_CHANNEL] < js->yaw_channel_duty_low)
	{
		js->yaw_channel_value = -1.0f;
	}
	if(st.duty_data[YAW_CHANNEL] > js->yaw_channel_duty_high)
	{
		js->yaw_channel_value = 1.0f;
	}
	else
	{
		js->yaw_channel_value = ((float)2.0 * (float)st.duty_data[YAW_CHANNEL] - (float)yaw_channel_limits_sum)/(float)yaw_channel_limits_range;
	}

	if(st.duty_data[VERTICAL_CHANNEL] < js->vertical_channel_duty_low)
	{
		js->vertical_channel_value = -1.0f;
	}
	if(st.duty_data[VERTICAL_CHANNEL] > js->vertical_channel_duty_high)
	{
		js->vertical_channel_value = 1.0f;
	}
	else
	{
		js->vertical_channel_value = ((float)2.0 * (float)st.duty_data[VERTICAL_CHANNEL] - (float)vertical_channel_limits_sum)/(float)vertical_channel_limits_range;
	}

	/*
		Copy over validity flags:
	 */
	js->vertical_channel_validity = st.channel_states[VERTICAL_CHANNEL];
	js->roll_channel_validity = st.channel_states[ROLL_CHANNEL];
	js->pitch_channel_validity = st.channel_states[PITCH_CHANNEL];
	js->yaw_channel_validity = st.channel_states[YAW_CHANNEL];
}