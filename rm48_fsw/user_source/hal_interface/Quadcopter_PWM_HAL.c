#include "Quadcopter_PWM_HAL.h"

/*----------------------------------------------------------------------------*/
/* Global variables                                                           */
/* Borrowed from TI's HET Halcogen-generated code, to shut GCC up on errors...*/

static float pwm0_pwm_val, pwm1_pwm_val, pwm2_pwm_val, pwm3_pwm_val;

static const uint32 s_het1pwmPolarity[8U] =
{
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
};

static const uint32 s_het2pwmPolarity[8U] =
{
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
};

/*
	Sets the appropriate PWM channel on the RM48's HET (High-end-timer) to
	specified duty cycle represented by pwmDuty (a floating-point value from 0 to 1)
 */
static void pwmSetDuty_HighPrecision(hetRAMBASE_t * hetRAM, uint32 pwm, float pwmDuty)
{
    uint32 action;
    uint32 pwmPolarity;
    float64   pwmPeriod = hetRAM->Instruction[(pwm << 1U) + 42U].Data + 128U;
    if(hetRAM == hetRAM1)
    {
        pwmPolarity = s_het1pwmPolarity[pwm];
    }
    else
    {
        pwmPolarity = s_het2pwmPolarity[pwm];
    }
    if (pwmDuty == 0.0f)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (pwmDuty >= 1.0f)
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control) & (~(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = (uint32)(pwmPeriod * pwmDuty) + 128U;
}

// This configures all vehicle PWM channels:
void QuadRotor_PWM_init(void)
{
	hetInit();
    pwm0_pwm_val = 0.0f;
    pwm1_pwm_val = 0.0f;
    pwm2_pwm_val = 0.0f;
    pwm3_pwm_val = 0.0f;
}

// Functions to initialize PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_start(void)
{
	pwmStart(MOTOR1_HET_CHANNEL);
}

void QuadRotor_motor2_start(void)
{
	pwmStart(MOTOR2_HET_CHANNEL);
}

void QuadRotor_motor3_start(void)
{
	pwmStart(MOTOR3_HET_CHANNEL);
}

void QuadRotor_motor4_start(void)
{
	pwmStart(MOTOR4_HET_CHANNEL);
}

// Functions to stop PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_stop(void)
{
	pwmStop(MOTOR1_HET_CHANNEL);
}

void QuadRotor_motor2_stop(void)
{
	pwmStop(MOTOR2_HET_CHANNEL);
}

void QuadRotor_motor3_stop(void)
{
	pwmStop(MOTOR3_HET_CHANNEL);
}

void QuadRotor_motor4_stop(void)
{
	pwmStop(MOTOR4_HET_CHANNEL);
}

// Functions to set PWM channels corresponding to individual motor/ESC's
// Duty is a value from 0.0 to 1.0 with 1.0 representing full throttle/duty cycle command to the ESC/motor:

void QuadRotor_motor1_setDuty(float duty)
{
    float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	pwmSetDuty_HighPrecision(MOTOR1_HET_CHANNEL, duty_scaled);
}

void QuadRotor_motor2_setDuty(float duty)
{
    float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	pwmSetDuty_HighPrecision(MOTOR2_HET_CHANNEL, duty_scaled);
}

void QuadRotor_motor3_setDuty(float duty)
{
    float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	pwmSetDuty_HighPrecision(MOTOR3_HET_CHANNEL, duty_scaled);
}

void QuadRotor_motor4_setDuty(float duty)
{
    float duty_scaled = (MAX_DUTY-MIN_DUTY)*duty + MIN_DUTY;
	pwmSetDuty_HighPrecision(MOTOR4_HET_CHANNEL, duty_scaled);
}

void QuadRotor_motor1_setDuty_raw(float duty)
{
    pwmSetDuty_HighPrecision(MOTOR1_HET_CHANNEL, duty);
}

void QuadRotor_motor2_setDuty_raw(float duty)
{
    pwmSetDuty_HighPrecision(MOTOR2_HET_CHANNEL, duty);
}

void QuadRotor_motor3_setDuty_raw(float duty)
{
    pwmSetDuty_HighPrecision(MOTOR3_HET_CHANNEL, duty);
}

void QuadRotor_motor4_setDuty_raw(float duty)
{
    pwmSetDuty_HighPrecision(MOTOR4_HET_CHANNEL, duty);
}
