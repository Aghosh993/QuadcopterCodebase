#include "pwm_input_hal.h"

static uint32_t het_edges_since_last_check;

static void pwmGetSignalHigherPrecision(hetRAMBASE_t * hetRAM, uint32 cap, pwm_info_t *pwm)
{
    uint32    pwmDuty   = (hetRAM->Instruction[(cap << 1U) + 25U].Data) >> 7U;
    uint32    pwmPeriod = (hetRAM->Instruction[(cap << 1U) + 26U].Data) >> 7U;

    pwm->duty_us   = pwmDuty;

    if( hetRAM == hetRAM1)
    {
        pwm->period_us = (uint32_t)(((float64)pwmPeriod * 1292.799F) / 1000.0F);
    }
    else
    {
        pwm->period_us = (uint32_t)(((float64)pwmPeriod * 1292.799F) / 1000.0F);
    }
}

void pwm_input_init(void)
{
    // hetInit();
    het_edges_since_last_check = RC_LOS_THRESHOLD;
}

void get_rc_state(rc_input_state* ret)
{
    pwm_info_t hetData[5];

    pwmGetSignalHigherPrecision(hetRAM1, cap0, &hetData[0]);
    pwmGetSignalHigherPrecision(hetRAM1, cap1, &hetData[1]);
    pwmGetSignalHigherPrecision(hetRAM2, cap0, &hetData[2]);
    pwmGetSignalHigherPrecision(hetRAM2, cap1, &hetData[3]);
    pwmGetSignalHigherPrecision(hetRAM1, cap2, &hetData[4]);

    uint8_t i = 0U;
    for(i=0U; i<5U; ++i)
    {
        ret->duty_data[i] = hetData[i].duty_us;
        ret->period_data[i] = hetData[i].period_us;
        if(het_edges_since_last_check < RC_LOS_THRESHOLD)
        {
            ret->channel_states[i] = CHANNEL_INVALID;                        
        }
        else
        {
            ret->channel_states[i] = CHANNEL_VALID;
        }
    }
}

void setup_user_feedback_gpios(void)
{
    sys_led_init();
}

void rc_input_validity_watchdog_callback(void)
{
    het_edges_since_last_check = edgeGetCounter(hetRAM2, edge0);
    edgeResetCounter(hetRAM2, edge0);
}