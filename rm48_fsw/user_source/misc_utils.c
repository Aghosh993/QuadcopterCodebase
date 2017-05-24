/*
	File: misc_utils.c

	(c) Abhimanyu Ghosh, 2016
 */
#include "misc_utils.h"
#include "mission_timekeeper.h"

/*
 * Simple blocking delay. Normally not used, instead the timekeeper_delay() function in
 * mission_timekeeper module is used due to tighter tolerance and accuracy (down to 1 millisecond).
 * However, that function requires interrupts to be enabled, and cannot run within an ISR.
 */

void insert_delay(int ms)
{
	int i = 0;
	int j = 0;
	for(i=0;i<ms;++i)
	{
		for(j=0;j<6250;++j)
		{
			++j;
		}
	}
}

void blocking_sci_SendData(uint8_t *dataptr, uint8_t len)
{
	uint8_t i = 0U;
	for(i=0; i<len; ++i)
	{
		sciSendByte(sciREG, dataptr[i]);
	}
}

void esc_cal(int motor)
{
	float max_pulse_ms = 0.8f; //0.85f
	uint32_t i = 0U;
	uint32_t steps = 500U;

	float duty = 0.9f;

	QuadRotor_PWM_init();

	switch(motor)
	{
		case 1:
			QuadRotor_motor1_start();
			QuadRotor_motor1_setDuty_raw(max_pulse_ms);
			break;
		case 2:
			QuadRotor_motor2_start();
			QuadRotor_motor2_setDuty_raw(max_pulse_ms);
			break;
		case 3:
			QuadRotor_motor3_start();
			QuadRotor_motor3_setDuty_raw(max_pulse_ms);
			break;
		case 4:
			QuadRotor_motor4_start();
			QuadRotor_motor4_setDuty_raw(max_pulse_ms);
			break;
		default:
			break;
	}

	timekeeper_delay(2000U);

	for(i=0U; i<steps; ++i)
	{
		duty = max_pulse_ms - (float)i*(float)0.4f/(float)steps;
		switch(motor)
			{
				case 1:
					QuadRotor_motor1_setDuty_raw(duty);
					break;
				case 2:
					QuadRotor_motor2_setDuty_raw(duty);
					break;
				case 3:
					QuadRotor_motor3_setDuty_raw(duty);
					break;
				case 4:
					QuadRotor_motor4_setDuty_raw(duty);
					break;
				default:
					break;
			}
		timekeeper_delay(3U);
	}
}
