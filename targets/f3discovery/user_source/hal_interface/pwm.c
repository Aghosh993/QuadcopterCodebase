#include "pwm.h"
#include "vehicleMixer.h"

static volatile TIM_HandleTypeDef tim_init;
static volatile TIM_OC_InitTypeDef tim1_conf;
static volatile bool interruptFlag;

void init_pwm(void)
{
	interruptFlag = false;
	tim_init.Instance = TIM1;
	tim_init.Channel = HAL_TIM_ACTIVE_CHANNEL_1 | HAL_TIM_ACTIVE_CHANNEL_2 | HAL_TIM_ACTIVE_CHANNEL_3 | HAL_TIM_ACTIVE_CHANNEL_4;
	
	tim_init.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim_init.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	tim_init.Init.Prescaler = 2;
	tim_init.Init.Period = TIMER_PERIOD_500HZ;
	tim_init.Init.AutoReloadPreload = 0;

	TIM_ClockConfigTypeDef tim_clk_init;
	tim_clk_init.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	tim_clk_init.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;

	HAL_TIM_PWM_Init(&tim_init);

	tim1_conf.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	tim1_conf.OCMode = TIM_OCMODE_PWM1; // Timer PWM mode 2, if Counter > Pulse, we set the output pin LOW till end of cycle
	tim1_conf.OCNIdleState = TIM_OCIDLESTATE_RESET;
	tim1_conf.Pulse = 24000;

	HAL_TIM_PWM_ConfigChannel(&tim_init, &tim1_conf, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&tim_init, &tim1_conf, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&tim_init, &tim1_conf, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&tim_init, &tim1_conf, TIM_CHANNEL_4);

	/*
		The PreemptPriority value (i.e. 2nd arg in HAL_NVIC_SetPriority) 
		CANNOT be lower than the configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
		in FreeRTOSConfig.h!!
	*/
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0); 
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

	HAL_TIM_PWM_Start(&tim_init, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&tim_init, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&tim_init, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&tim_init, TIM_CHANNEL_4);
	__HAL_TIM_ENABLE_IT(&tim_init, TIM_IT_CC1);
}

void set_pwm(pwm_channel ch, float duty)
{
	switch(ch)
	{
		case pwm1:
			__HAL_TIM_SET_COMPARE(&tim_init, TIM_CHANNEL_4, 
									(uint32_t)(duty * (float)TIMER_PERIOD_500HZ));
			break;
		case pwm2:
			__HAL_TIM_SET_COMPARE(&tim_init, TIM_CHANNEL_3, 
									(uint32_t)(duty * (float)TIMER_PERIOD_500HZ));
			break;
		case pwm3:
			__HAL_TIM_SET_COMPARE(&tim_init, TIM_CHANNEL_2, 
									(uint32_t)(duty * (float)TIMER_PERIOD_500HZ));
			break;
		case pwm4:
			__HAL_TIM_SET_COMPARE(&tim_init, TIM_CHANNEL_1, 
									(uint32_t)(duty * (float)TIMER_PERIOD_500HZ));
			break;
	}
}

bool pwmReadyForNewCmd(void)
{
	bool ret = interruptFlag;
	interruptFlag = false;
	return ret;
}

void disablePwmIrq(void)
{
	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
}

void enablePwmIrq(void)
{
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/*
	IRQ def(s) to override weak default(s):
 */

void TIM1_CC_IRQHandler(void)
{
	if(__HAL_TIM_GET_FLAG(&tim_init, TIM_FLAG_CC1))
	{
		__HAL_TIM_CLEAR_FLAG(&tim_init, TIM_FLAG_CC1);
		interruptFlag = true; // Raise the signal that we're ready for a new set of
								// duty cycle commands
	}
}