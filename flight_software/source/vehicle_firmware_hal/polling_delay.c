#include "polling_delay.h"

/*
 * Simple blocking delay. Normally not used, instead the timekeeper_delay() function in
 * mission_timekeeper module is used due to tighter tolerance and accuracy (down to 1 millisecond).
 * However, that function requires interrupts to be enabled, and cannot run within an ISR.
 */

void insert_delay_ms(int ms)
{
	int i = 0;
	int j = 0;
	for(i=0;i<ms;++i)
	{
		for(j=0;j<CPU_CYCLES_PER_MS;++j)
		{
			++j;
		}
	}
}