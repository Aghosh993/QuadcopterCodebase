#include "pwm_input_hal.h"

static inline void _enableIRQ()
{
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM15_IRQn);
}

static inline void _disableIRQ()
{
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
  HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM15_IRQn);
}

static volatile rc_input_state current_state;

static volatile uint8_t rc_tim2_watchdog_counter;
static volatile uint8_t rc_tim3_watchdog_counter;
static volatile uint8_t rc_tim4_watchdog_counter;
static volatile uint8_t rc_tim8_watchdog_counter;
static volatile uint8_t rc_tim15_watchdog_counter;

static volatile uint8_t rc_tim2_edgeCount;
static volatile uint8_t rc_tim3_edgeCount;
static volatile uint8_t rc_tim4_edgeCount;
static volatile uint8_t rc_tim8_edgeCount;
static volatile uint8_t rc_tim15_edgeCount;

/*
  Private functions for hardware initialization:
 */

// TIM3, CH1:

static volatile TIM_HandleTypeDef tim3_init;
static volatile TIM_IC_InitTypeDef tim3_conf;
static volatile TIM_SlaveConfigTypeDef tim3_slave_conf;

static void init_ic_rc1(void)
{
  tim3_init.Instance = TIM3;  
  tim3_init.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim3_init.Init.ClockDivision = 0;
  tim3_init.Init.Prescaler = 72;
  tim3_init.Init.Period = 0xFFFF;

  HAL_TIM_IC_Init(&tim3_init);

  tim3_slave_conf.InputTrigger     = TIM_TS_TI1FP1;
  tim3_slave_conf.SlaveMode        = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  tim3_slave_conf.TriggerPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim3_slave_conf.TriggerPrescaler = TIM_ICPSC_DIV1;
  tim3_slave_conf.TriggerFilter    = 0;
  HAL_TIM_SlaveConfigSynchronization(&tim3_init, &tim3_slave_conf);

  tim3_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  tim3_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim3_conf.ICPrescaler = TIM_ICPSC_DIV1;
  tim3_conf.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&tim3_init, &tim3_conf, TIM_CHANNEL_1);

  tim3_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim3_conf.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&tim3_init, &tim3_conf, TIM_CHANNEL_2);


  HAL_TIM_IC_Start_IT(&tim3_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&tim3_init, TIM_CHANNEL_1);

  HAL_TIM_IC_Start(&tim3_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&tim3_init, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_IT(&tim3_init, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&tim3_init, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&tim3_init, TIM_IT_UPDATE);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// TIM4, CH1:

static volatile TIM_HandleTypeDef tim4_init;
static volatile TIM_IC_InitTypeDef tim4_conf;
static volatile TIM_SlaveConfigTypeDef tim4_slave_conf;

static void init_ic_rc2(void)
{
  tim4_init.Instance = TIM4;  
  tim4_init.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim4_init.Init.ClockDivision = 0;
  tim4_init.Init.Prescaler = 72;
  tim4_init.Init.Period = 0xFFFF;

  HAL_TIM_IC_Init(&tim4_init);

  tim4_slave_conf.InputTrigger     = TIM_TS_TI1FP1;
  tim4_slave_conf.SlaveMode        = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  tim4_slave_conf.TriggerPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim4_slave_conf.TriggerPrescaler = TIM_ICPSC_DIV1;
  tim4_slave_conf.TriggerFilter    = 0;
  HAL_TIM_SlaveConfigSynchronization(&tim4_init, &tim4_slave_conf);

  tim4_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  tim4_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim4_conf.ICPrescaler = TIM_ICPSC_DIV1;
  tim4_conf.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&tim4_init, &tim4_conf, TIM_CHANNEL_1);

  tim4_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim4_conf.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&tim4_init, &tim4_conf, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&tim4_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&tim4_init, TIM_CHANNEL_1);

  HAL_TIM_IC_Start(&tim4_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&tim4_init, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_IT(&tim4_init, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&tim4_init, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&tim4_init, TIM_IT_UPDATE);

  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

// TIM8, CH1:

static volatile TIM_HandleTypeDef tim8_init;
static volatile TIM_IC_InitTypeDef tim8_conf;
static volatile TIM_SlaveConfigTypeDef tim8_slave_conf;

static void init_ic_rc3(void)
{
  tim8_init.Instance = TIM8;  
  tim8_init.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim8_init.Init.ClockDivision = 0;
  tim8_init.Init.Prescaler = 72;
  tim8_init.Init.Period = 0xFFFF;

  HAL_TIM_IC_Init(&tim8_init);

  tim8_slave_conf.InputTrigger     = TIM_TS_TI1FP1;
  tim8_slave_conf.SlaveMode        = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  tim8_slave_conf.TriggerPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim8_slave_conf.TriggerPrescaler = TIM_ICPSC_DIV1;
  tim8_slave_conf.TriggerFilter    = 0;
  HAL_TIM_SlaveConfigSynchronization(&tim8_init, &tim8_slave_conf);

  tim8_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  tim8_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim8_conf.ICPrescaler = TIM_ICPSC_DIV1;
  tim8_conf.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&tim8_init, &tim8_conf, TIM_CHANNEL_1);

  tim8_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim8_conf.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&tim8_init, &tim8_conf, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&tim8_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&tim8_init, TIM_CHANNEL_1);

  HAL_TIM_IC_Start(&tim8_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&tim8_init, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_IT(&tim8_init, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&tim8_init, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&tim8_init, TIM_IT_UPDATE);

  HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 1);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
}

// TIM2, CH1:

static volatile TIM_HandleTypeDef tim2_init;
static volatile TIM_IC_InitTypeDef tim2_conf;
static volatile TIM_SlaveConfigTypeDef tim2_slave_conf;

static void init_ic_rc4(void)
{
  tim2_init.Instance = TIM2;  
  tim2_init.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim2_init.Init.ClockDivision = 0;
  tim2_init.Init.Prescaler = 72;
  tim2_init.Init.Period = 0xFFFF;

  HAL_TIM_IC_Init(&tim2_init);

  tim2_slave_conf.InputTrigger     = TIM_TS_TI1FP1;
  tim2_slave_conf.SlaveMode        = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  tim2_slave_conf.TriggerPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim2_slave_conf.TriggerPrescaler = TIM_ICPSC_DIV1;
  tim2_slave_conf.TriggerFilter    = 0;
  HAL_TIM_SlaveConfigSynchronization(&tim2_init, &tim2_slave_conf);

  tim2_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  tim2_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim2_conf.ICPrescaler = TIM_ICPSC_DIV1;
  tim2_conf.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&tim2_init, &tim2_conf, TIM_CHANNEL_1);

  tim2_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim2_conf.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&tim2_init, &tim2_conf, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&tim2_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&tim2_init, TIM_CHANNEL_1);

  HAL_TIM_IC_Start(&tim2_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&tim2_init, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_IT(&tim2_init, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&tim2_init, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&tim2_init, TIM_IT_UPDATE);

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

// TIM15, CH1:

static volatile TIM_HandleTypeDef tim15_init;
static volatile TIM_IC_InitTypeDef tim15_conf;
static volatile TIM_SlaveConfigTypeDef tim15_slave_conf;

static void init_ic_rc5(void)
{
  tim15_init.Instance = TIM15;  
  tim15_init.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim15_init.Init.ClockDivision = 0;
  tim15_init.Init.Prescaler = 72;
  tim15_init.Init.Period = 0xFFFF;

  HAL_TIM_IC_Init(&tim15_init);

  tim15_slave_conf.InputTrigger     = TIM_TS_TI1FP1;
  tim15_slave_conf.SlaveMode        = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  tim15_slave_conf.TriggerPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim15_slave_conf.TriggerPrescaler = TIM_ICPSC_DIV1;
  tim15_slave_conf.TriggerFilter    = 0;
  HAL_TIM_SlaveConfigSynchronization(&tim15_init, &tim15_slave_conf);

  tim15_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  tim15_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim15_conf.ICPrescaler = TIM_ICPSC_DIV1;
  tim15_conf.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&tim15_init, &tim15_conf, TIM_CHANNEL_1);

  tim15_conf.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  tim15_conf.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&tim15_init, &tim15_conf, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&tim15_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&tim15_init, TIM_CHANNEL_1);

  HAL_TIM_IC_Start(&tim15_init, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&tim15_init, TIM_CHANNEL_1);

  __HAL_TIM_ENABLE_IT(&tim15_init, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&tim15_init, TIM_IT_CC2);
  __HAL_TIM_ENABLE_IT(&tim15_init, TIM_IT_UPDATE);

  HAL_NVIC_SetPriority(TIM15_IRQn, 0, 1);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM15_IRQn);
}

/*
  Interrupt Handlers
  Implementations of ST Cube HAL weakly-defined functions:
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      current_state.duty_data[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      current_state.period_data[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    rc_tim2_watchdog_counter += 1U;
  }

  if(htim->Instance == TIM3)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      current_state.duty_data[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      current_state.period_data[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    rc_tim3_watchdog_counter += 1U;
  }

  if(htim->Instance == TIM4)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      current_state.duty_data[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      current_state.period_data[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    rc_tim4_watchdog_counter += 1U;
  }

  if(htim->Instance == TIM8)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      current_state.duty_data[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      current_state.period_data[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    rc_tim8_watchdog_counter += 1U;
  }


  if(htim->Instance == TIM15)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      current_state.duty_data[4] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      current_state.period_data[4] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    rc_tim15_watchdog_counter += 1U;
  }
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim2_init);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim3_init);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim4_init);
}

void TIM8_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim8_init);
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim15_init);
}

/*
  Exported functions:
 */

void pwm_input_hal_init(void)
{
  rc_tim2_edgeCount = 0U;
  rc_tim3_edgeCount = 0U;
  rc_tim4_edgeCount = 0U;
  rc_tim8_edgeCount = 0U;
  rc_tim15_edgeCount = 0U;

  init_ic_rc1();
  init_ic_rc2();
  init_ic_rc3();
  init_ic_rc4();
  init_ic_rc5();

  uint8_t i = 0U;
  for(i=0U; i<5U; i++)
  {
    current_state.duty_data[i] = 0U;
    current_state.period_data[i] = 0U;
  }

  rc_tim2_watchdog_counter = RC_TIMEOUT_THRESHOLD;
  rc_tim3_watchdog_counter = RC_TIMEOUT_THRESHOLD;
  rc_tim4_watchdog_counter = RC_TIMEOUT_THRESHOLD;
  rc_tim8_watchdog_counter = RC_TIMEOUT_THRESHOLD;
  rc_tim15_watchdog_counter = RC_TIMEOUT_THRESHOLD;
}

void get_rc_state(rc_input_state* ret)
{
  uint8_t i = 0U;

  _disableIRQ();

  for(i=0U; i<5U; ++i)
  {
    ret->period_data[i] = current_state.period_data[i];
    ret->duty_data[i] = current_state.duty_data[i];
  }
  if(rc_tim2_edgeCount < RC_TIMEOUT_THRESHOLD)
  {
    ret->channel_states[3] = CHANNEL_INVALID;
  }
  else
  {
    ret->channel_states[3] = CHANNEL_VALID;
  }

  if(rc_tim3_edgeCount < RC_TIMEOUT_THRESHOLD)
  {
    ret->channel_states[0] = CHANNEL_INVALID;
  }
  else
  {
    ret->channel_states[0] = CHANNEL_VALID;
  }

  if(rc_tim4_edgeCount < RC_TIMEOUT_THRESHOLD)
  {
    ret->channel_states[1] = CHANNEL_INVALID;
  }
  else
  {
    ret->channel_states[1] = CHANNEL_VALID;
  }

  if(rc_tim8_edgeCount < RC_TIMEOUT_THRESHOLD)
  {
    ret->channel_states[2] = CHANNEL_INVALID;
  }
  else
  {
    ret->channel_states[2] = CHANNEL_VALID;
  }

  if(rc_tim15_edgeCount < RC_TIMEOUT_THRESHOLD)
  {
    ret->channel_states[4] = CHANNEL_INVALID;
  }
  else
  {
    ret->channel_states[4] = CHANNEL_VALID;
  }

  _enableIRQ();
}

void rc_input_watchdog_callback(void)
{
  rc_tim2_edgeCount = rc_tim2_watchdog_counter;
  rc_tim2_watchdog_counter = 0U;

  rc_tim3_edgeCount = rc_tim3_watchdog_counter;
  rc_tim3_watchdog_counter = 0U;

  rc_tim4_edgeCount = rc_tim4_watchdog_counter;
  rc_tim4_watchdog_counter = 0U;

  rc_tim8_edgeCount = rc_tim8_watchdog_counter;
  rc_tim8_watchdog_counter = 0U;

  rc_tim15_edgeCount = rc_tim15_watchdog_counter;
  rc_tim15_watchdog_counter = 0U;
}