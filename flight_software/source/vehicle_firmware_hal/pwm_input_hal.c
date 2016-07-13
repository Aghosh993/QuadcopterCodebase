#include <pwm_input_hal.h>

/*
	PD3 -> 	Timer2 Input capture
	PC6 -> 	Timer3 Input capture
	PD12 -> Timer4 Input capture
	PA15 -> Timer8 Input capture
 */

static volatile rc_input_state current_state;

static volatile uint8_t rc_tim2_watchdog_counter;
static volatile uint8_t rc_tim3_watchdog_counter;
static volatile uint8_t rc_tim4_watchdog_counter;
static volatile uint8_t rc_tim8_watchdog_counter;

void pwm_input_init(void)
{
	setup_timer2_input_capture();
	setup_timer3_input_capture();
	setup_timer4_input_capture();
	setup_timer8_input_capture();
	
	uint8_t i = 0U;
	for(i=0U;i<4U;++i)
	{
		current_state.duty_data[i] = 0U;
		current_state.period_data[i] = 0U;
	}

	rc_tim2_watchdog_counter = RC_TIMEOUT_THRESHOLD;
	rc_tim3_watchdog_counter = RC_TIMEOUT_THRESHOLD;
	rc_tim4_watchdog_counter = RC_TIMEOUT_THRESHOLD;
	rc_tim8_watchdog_counter = RC_TIMEOUT_THRESHOLD;
}

void setup_timer2_input_capture(void)
{	
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_TIM2);

	timer_reset(TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM2, 0xFFFF);
	timer_set_prescaler(TIM2, 63); // For 1 MHz Timer clock, 1 counts/us
	timer_enable_counter(TIM2);

	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO3);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	gpio_set_af(GPIOD, GPIO_AF2, GPIO3);

	timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI1);

	timer_ic_set_polarity(TIM2, TIM_IC1, TIM_IC_RISING);
	timer_ic_set_polarity(TIM2, TIM_IC2, TIM_IC_FALLING);

	timer_slave_set_trigger(TIM2, TIM_SMCR_TS_IT1FP1);
	timer_slave_set_mode(TIM2, TIM_SMCR_SMS_RM);

	nvic_set_priority(NVIC_TIM2_IRQ, 2);
	nvic_enable_irq(NVIC_TIM2_IRQ);

	timer_enable_irq(TIM2, TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE);

	timer_ic_enable(TIM2, TIM_IC1);
	timer_ic_enable(TIM2, TIM_IC2);	
}

void setup_timer3_input_capture(void)
{	
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_TIM3);

	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM3, 0xFFFF);
	timer_set_prescaler(TIM3, 63); // For 1 MHz Timer clock, 1 counts/us
	timer_enable_counter(TIM3);

	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO6);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOC, GPIO_AF2, GPIO6);

	timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI1);

	timer_ic_set_polarity(TIM3, TIM_IC1, TIM_IC_RISING);
	timer_ic_set_polarity(TIM3, TIM_IC2, TIM_IC_FALLING);

	timer_slave_set_trigger(TIM3, TIM_SMCR_TS_IT1FP1);
	timer_slave_set_mode(TIM3, TIM_SMCR_SMS_RM);

	nvic_set_priority(NVIC_TIM3_IRQ, 2);
	nvic_enable_irq(NVIC_TIM3_IRQ);

	timer_enable_irq(TIM3, TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE);

	timer_ic_enable(TIM3, TIM_IC1);
	timer_ic_enable(TIM3, TIM_IC2);	
}

void setup_timer4_input_capture(void)
{	
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_TIM4);

	timer_reset(TIM4);
	timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM4, 0xFFFF);
	timer_set_prescaler(TIM4, 63); // For 1 MHz Timer clock, 1 counts/us
	timer_enable_counter(TIM4);

	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO12);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
	gpio_set_af(GPIOD, GPIO_AF2, GPIO12);

	timer_ic_set_input(TIM4, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM4, TIM_IC2, TIM_IC_IN_TI1);

	timer_ic_set_polarity(TIM4, TIM_IC1, TIM_IC_RISING);
	timer_ic_set_polarity(TIM4, TIM_IC2, TIM_IC_FALLING);

	timer_slave_set_trigger(TIM4, TIM_SMCR_TS_IT1FP1);
	timer_slave_set_mode(TIM4, TIM_SMCR_SMS_RM);

	nvic_set_priority(NVIC_TIM4_IRQ, 2);
	nvic_enable_irq(NVIC_TIM4_IRQ);

	timer_enable_irq(TIM4, TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE);

	timer_ic_enable(TIM4, TIM_IC1);
	timer_ic_enable(TIM4, TIM_IC2);	
}

void setup_timer8_input_capture(void)
{	
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM8);

	timer_reset(TIM8);
	timer_set_mode(TIM8, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM8, 0xFFFF);
	timer_set_prescaler(TIM8, 63); // For 1 MHz Timer clock, 1 counts/us
	timer_enable_counter(TIM8);

	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO15);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO15);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO15);

	timer_ic_set_input(TIM8, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM8, TIM_IC2, TIM_IC_IN_TI1);

	timer_ic_set_polarity(TIM8, TIM_IC1, TIM_IC_RISING);
	timer_ic_set_polarity(TIM8, TIM_IC2, TIM_IC_FALLING);

	timer_slave_set_trigger(TIM8, TIM_SMCR_TS_IT1FP1);
	timer_slave_set_mode(TIM8, TIM_SMCR_SMS_RM);

	nvic_set_priority(NVIC_TIM8_CC_IRQ, 2);
	nvic_enable_irq(NVIC_TIM8_CC_IRQ);

	timer_enable_irq(TIM8, TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE);

	timer_ic_enable(TIM8, TIM_IC1);
	timer_ic_enable(TIM8, TIM_IC2);	
}

void timer2_isr_callback(void)
{
	if((TIM2_SR & TIM_SR_UIF) != 0)
	{
		timer_clear_flag(TIM2, TIM_SR_UIF);		
	}
	if ((TIM2_SR & TIM_SR_CC2IF) != 0)
	{
		timer_clear_flag(TIM2, TIM_SR_CC2IF);
		current_state.duty_data[0] = TIM2_CCR2;
	}
	if ((TIM2_SR & TIM_SR_CC1IF) != 0)
	{
		timer_clear_flag(TIM2, TIM_SR_CC1IF);
		current_state.period_data[0] = TIM2_CCR1;
		/*
			Kick the dog:
		 */
		rc_tim2_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[0] = CHANNEL_VALID;
	}
}

void timer3_isr_callback(void)
{
	if((TIM3_SR & TIM_SR_UIF) != 0)
	{
		timer_clear_flag(TIM3, TIM_SR_UIF);		
	}
	if ((TIM3_SR & TIM_SR_CC2IF) != 0)
	{
		timer_clear_flag(TIM3, TIM_SR_CC2IF);
		current_state.duty_data[1] = TIM3_CCR2;
	}
	if ((TIM3_SR & TIM_SR_CC1IF) != 0)
	{
		timer_clear_flag(TIM3, TIM_SR_CC1IF);
		current_state.period_data[1] = TIM3_CCR1;
		/*
			Kick the dog:
		 */
		rc_tim3_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[1] = CHANNEL_VALID;
	}
}

void timer4_isr_callback(void)
{
	if((TIM4_SR & TIM_SR_UIF) != 0)
	{
		timer_clear_flag(TIM4, TIM_SR_UIF);		
	}
	if ((TIM4_SR & TIM_SR_CC2IF) != 0)
	{
		timer_clear_flag(TIM4, TIM_SR_CC2IF);
		current_state.duty_data[2] = TIM4_CCR2;
	}
	if ((TIM4_SR & TIM_SR_CC1IF) != 0)
	{
		timer_clear_flag(TIM4, TIM_SR_CC1IF);
		current_state.period_data[2] = TIM4_CCR1;
		/*
			Kick the dog:
		 */
		rc_tim4_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[2] = CHANNEL_VALID;
	}
}

void timer8_isr_callback(void)
{
	if ((TIM8_SR & TIM_SR_CC2IF) != 0)
	{
		timer_clear_flag(TIM8, TIM_SR_CC2IF);
		current_state.duty_data[3] = TIM8_CCR2;
	}
	if ((TIM8_SR & TIM_SR_CC1IF) != 0)
	{
		timer_clear_flag(TIM8, TIM_SR_CC1IF);
		current_state.period_data[3] = TIM8_CCR1;
		/*
			Kick the dog:
		 */
		rc_tim8_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[3] = CHANNEL_VALID;
	}
}

void rc_input_watchdog_callback(void)
{
	/*
		If any of the channels have exceeded max update interval, safe the channel(s):
	 */
	if(rc_tim2_watchdog_counter == 0U)
	{
		rc_tim2_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[0] = CHANNEL_INVALID;
	}
	else
	{
		--rc_tim2_watchdog_counter;
	}
	if(rc_tim3_watchdog_counter == 0U)
	{
		rc_tim3_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[1] = CHANNEL_INVALID;
	}
	else
	{
		--rc_tim3_watchdog_counter;
	}
	if(rc_tim4_watchdog_counter == 0U)
	{
		rc_tim4_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[2] = CHANNEL_INVALID;
	}
	else
	{
		--rc_tim4_watchdog_counter;
	}
	if(rc_tim8_watchdog_counter == 0U)
	{
		rc_tim8_watchdog_counter = RC_TIMEOUT_THRESHOLD;
		current_state.channel_states[3] = CHANNEL_INVALID;
	}
	else
	{
		--rc_tim8_watchdog_counter;
	}
}

void get_rc_state(rc_input_state* ret)
{
	uint8_t i = 0U;

	_disable_interrupts();

	// ret->period_data[0] = current_state.period_data[0];
	// ret->duty_data[0] = current_state.duty_data[0];

	// ret->period_data[1] = current_state.period_data[1];
	// ret->duty_data[1] = current_state.duty_data[1];

	// ret->period_data[2] = current_state.period_data[2];
	// ret->duty_data[2] = current_state.duty_data[2];

	// ret->period_data[3] = current_state.period_data[3];
	// ret->duty_data[3] = current_state.duty_data[3];
	for(i=0U; i<4U; ++i)
	{
		ret->period_data[i] = current_state.period_data[i];
		ret->duty_data[i] = current_state.duty_data[i];
		ret->channel_states[i] = current_state.channel_states[i];
	}

	_enable_interrupts();
}