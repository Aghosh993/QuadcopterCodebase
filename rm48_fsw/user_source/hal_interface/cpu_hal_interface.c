#include "cpu_hal_interface.h"

/*
	A simple error handler that just blocks indefinitely until a HW reset:
 */

static void Error_Handler(void)
{
	while(1);
}

void cpu_init(void)
{
	/*
		Initialize main PLL, clock tree, safety init:
	 */
	systemInit();
	
	/*
		Pinmux setup:
	 */
	muxInit();

	/*
		Set up Halcogen-derived vector table and priorities:
	 */
	vimInit();

	/*
	Set up 1 kHz RTI interrupt:
	*/
	rtiInit();
	rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
	rtiStartCounter(rtiCOUNTER_BLOCK0);
}

void cpu_sw_delay(uint32_t ms)
{
	uint32_t i = 0U;
	uint32_t j = 0U;

	for(i = 0U; i < ms; ++i)
	{
		for(j = 0U; j < CPU_TICKS_PER_MS; ++j)
		{
			asm("nop");
		}
	}
}