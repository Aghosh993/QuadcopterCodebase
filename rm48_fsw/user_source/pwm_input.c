#include <pwm_input.h>

extern rt_telemetry_comm_channel telem0;

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

void init_rc_inputs(rc_joystick_data_struct* js, int calibrate)
{
	pwm_input_init();
	#ifdef PROVIDE_LED_USER_FEEDBACK
		board_led_on(CAL_SUCCESS_LED);
		board_led_on(ERROR_LED);
	#endif

	if(calibrate)
	{
		do_rc_channel_callibration(js, ROLL_CHANNEL);
		do_rc_channel_callibration(js, PITCH_CHANNEL);
		do_rc_channel_callibration(js, YAW_CHANNEL);
		do_rc_channel_callibration(js, VERTICAL_CHANNEL);
		do_rc_channel_callibration(js, CHANNEL5_MODE_SWITCH);
	}
	else
	{
		js->roll_channel_duty_low = 821;
		js->roll_channel_duty_high = 1469;
		js->pitch_channel_duty_low = 832;
		js->pitch_channel_duty_high = 1455;
		js->yaw_channel_duty_low = 820;
		js->yaw_channel_duty_high = 1460;
		js->vertical_channel_duty_low = 821;
		js->vertical_channel_duty_high = 1471;
		js->mode_switch_channel_duty_low = 821;
		js->mode_switch_channel_duty_high = 1472;
	}

	js->roll_channel_value = 0.0f;
	js->pitch_channel_value = 0.0f;
	js->yaw_channel_value = 0.0f;
	js->vertical_channel_value = -1.0f;
	js->mode_switch_channel_value = 0.0f;
}

void do_rc_channel_callibration(rc_joystick_data_struct* js, uint8_t axis)
{
	uint8_t samples_iterator = 0U;
	uint32_t averaging_sum = 0U;
	uint8_t i = 0U;

	int32_t telem_out = 0;

	#ifdef PWM_INPUT_DEBUG_OVER_UART
		telem_out = (int32_t)axis;
		send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"RC_CAL", 6, &telem_out, 1);
	#endif
	rc_input_state st;
	get_rc_state(&st);
	/*
		Block until all RC inputs are sane (i.e. within expected bounds for RC pulse lengths):
	 */
	board_led_off(CAL_SUCCESS_LED);
	board_led_on(ERROR_LED);

	while(rc_duty_isOutOfBounds(st.duty_data[axis]))
	{
		get_rc_state(&st);
	}
	#ifdef PWM_INPUT_DEBUG_OVER_UART
		send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"RC Input sane on this channel, proceeding\r\n", 43);
	#endif
	board_led_off(ERROR_LED);

	/*
		Low limit calibration:
	 */

	#ifdef PWM_INPUT_DEBUG_OVER_UART
	switch(axis)
	{
		case ROLL_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating roll stick low\r\n", 28);
			break;
		case PITCH_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating pitch stick low\r\n", 29);
			break;
		case YAW_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating yaw stick low\r\n", 27);
			break;
		case VERTICAL_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating vertical stick low\r\n", 32);
			break;
		case CHANNEL5_MODE_SWITCH:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating mode switch low\r\n", 29);
			break;
	}
	#endif

	/*
		Wait for user to move yaw stick to left limit:
	 */
	board_led_on(ERROR_LED);
	while(!rc_duty_isNearNominalLimit(st.duty_data[axis], RC_LOW_DUTY_NOMINAL_VALUE))
	{
		get_rc_state(&st);
	}
	board_led_off(ERROR_LED);
	/* 	
	 	Wait 500 milliseconds for input to stabilize 
	 	and user to reach stick end stop:
	 */
	timekeeper_delay(500U);

	#ifdef PWM_INPUT_DEBUG_OVER_UART
		send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Obtaining samples now\r\n", 23);
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
		case CHANNEL5_MODE_SWITCH:
			js->mode_switch_channel_duty_low = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
	}

	/*
		Success!
	 */
	#ifdef PWM_INPUT_DEBUG_OVER_UART
		switch(axis)
		{
			case ROLL_CHANNEL:
				telem_out = (int32_t)(js->roll_channel_duty_low);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"ROLL_L", 6, &telem_out, 1);
				break;
			case PITCH_CHANNEL:
				telem_out = (int32_t)(js->pitch_channel_duty_low);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"PIT_L", 5, &telem_out, 1);
				break;
			case YAW_CHANNEL:
				telem_out = (int32_t)(js->yaw_channel_duty_low);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"YAW_L", 5, &telem_out, 1);
				break;
			case VERTICAL_CHANNEL:
				telem_out = (int32_t)(js->vertical_channel_duty_low);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"VERT_L", 6, &telem_out, 1);
				break;
			case CHANNEL5_MODE_SWITCH:
				telem_out = (int32_t)(js->mode_switch_channel_duty_low);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"MODE_L", 6, &telem_out, 1);
				break;
		}
	#endif
	for(i = 0U; i < 2U; ++i)
	{
		board_led_on(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
		board_led_off(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
	}

	averaging_sum = 0U;

	/*
		High limit calibration:
	 */

	#ifdef PWM_INPUT_DEBUG_OVER_UART
	switch(axis)
	{
		case ROLL_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating roll stick high\r\n", 29);
			break;
		case PITCH_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating pitch stick high\r\n", 30);
			break;
		case YAW_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating yaw stick high\r\n", 28);
			break;
		case VERTICAL_CHANNEL:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating vertical stick high\r\n", 33);
			break;
		case CHANNEL5_MODE_SWITCH:
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Calibrating mode switch high\r\n", 30);
			break;
	}
	#endif
	/*
		Wait for user to move yaw stick to left limit:
	 */
	board_led_on(ERROR_LED);
	while(!rc_duty_isNearNominalLimit(st.duty_data[axis], RC_HIGH_DUTY_NOMINAL_VALUE))
	{
		get_rc_state(&st);
	}
	board_led_off(ERROR_LED);
	/* 	
	 	Wait 500 milliseconds for input to stabilize 
	 	and user to reach stick end stop:
	 */
	timekeeper_delay(500U);

	#ifdef PWM_INPUT_DEBUG_OVER_UART
		send_telem_msg_string_blocking(&telem0, (uint8_t *)"RC_CAL", 6, (uint8_t *)"Obtaining samples now\r\n", 23);
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
		case CHANNEL5_MODE_SWITCH:
			js->mode_switch_channel_duty_high = (uint32_t)((float)averaging_sum/(float)N_SAMPLES_RC_SIGNAL);
			break;
	}

	/*
		Success!
	 */
	#ifdef PWM_INPUT_DEBUG_OVER_UART
		switch(axis)
		{
			case ROLL_CHANNEL:
				telem_out = (int32_t)(js->roll_channel_duty_high);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"ROLL_H", 6, &telem_out, 1);
				break;
			case PITCH_CHANNEL:
				telem_out = (int32_t)(js->pitch_channel_duty_high);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"PIT_H", 5, &telem_out, 1);
				break;
			case YAW_CHANNEL:
				telem_out = (int32_t)(js->yaw_channel_duty_high);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"YAW_H", 5, &telem_out, 1);
				break;
			case VERTICAL_CHANNEL:
				telem_out = (int32_t)(js->vertical_channel_duty_high);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"VERT_H", 6, &telem_out, 1);
				break;
			case CHANNEL5_MODE_SWITCH:
				telem_out = (int32_t)(js->mode_switch_channel_duty_high);
				send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"MODE_H", 6, &telem_out, 1);
				break;
		}
	#endif
	for(i = 0U; i < 2U; ++i)
	{
		board_led_on(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
		board_led_off(CAL_SUCCESS_LED);
		timekeeper_delay(500U);
	}
}

void get_rc_input_values(rc_joystick_data_struct* js)
{
	uint32_t roll_channel_limits_range = js->roll_channel_duty_high-js->roll_channel_duty_low;
	uint32_t pitch_channel_limits_range = js->pitch_channel_duty_high-js->pitch_channel_duty_low;
	uint32_t yaw_channel_limits_range = js->yaw_channel_duty_high-js->yaw_channel_duty_low;
	uint32_t vertical_channel_limits_range = js->vertical_channel_duty_high-js->vertical_channel_duty_low;
	uint32_t mode_switch_channel_limits_range = js->mode_switch_channel_duty_high-js->mode_switch_channel_duty_low;

	uint32_t roll_channel_limits_sum = js->roll_channel_duty_high+js->roll_channel_duty_low;
	uint32_t pitch_channel_limits_sum = js->pitch_channel_duty_high+js->pitch_channel_duty_low;
	uint32_t yaw_channel_limits_sum = js->yaw_channel_duty_high+js->yaw_channel_duty_low;
	uint32_t vertical_channel_limits_sum = js->vertical_channel_duty_high+js->vertical_channel_duty_low;
	uint32_t mode_switch_channel_limits_sum = js->mode_switch_channel_duty_high+js->mode_switch_channel_duty_low;

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

	if(st.duty_data[CHANNEL5_MODE_SWITCH] < js->mode_switch_channel_duty_low)
	{
		js->mode_switch_channel_value = -1.0f;
	}
	if(st.duty_data[CHANNEL5_MODE_SWITCH] > js->mode_switch_channel_duty_high)
	{
		js->mode_switch_channel_value = 1.0f;
	}
	else
	{
		js->mode_switch_channel_value = ((float)2.0 * (float)st.duty_data[CHANNEL5_MODE_SWITCH] - (float)mode_switch_channel_limits_sum)/(float)mode_switch_channel_limits_range;
	}

	/*
		Copy over validity flags:
	 */
	js->vertical_channel_validity = st.channel_states[VERTICAL_CHANNEL];
	js->roll_channel_validity = st.channel_states[ROLL_CHANNEL];
	js->pitch_channel_validity = st.channel_states[PITCH_CHANNEL];
	js->yaw_channel_validity = st.channel_states[YAW_CHANNEL];
	js->mode_switch_channel_validity = st.channel_states[CHANNEL5_MODE_SWITCH];
}

channel5_mode_state get_ch5_mode(rc_joystick_data_struct js)
{
	if(js.mode_switch_channel_value < MODE5_NORMAL_THRESHOLD)
	{
		return MODE_NORMAL;
	}
	if(js.mode_switch_channel_value > MODE5_FAILSAFE_THRESHOLD)
	{
		return MODE_FAILSAFE;
	}
	return VALUE_ERR;
}