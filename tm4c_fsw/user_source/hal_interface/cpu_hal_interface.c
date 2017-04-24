#include "cpu_hal_interface.h"

/*
	A simple error handler that just blocks indefinitely until a HW reset:
 */

static void Error_Handler(void)
{
	while(1);
}

/*
  Set up system PLL and derived clocks. This is taken from TI example code.
 */
static void SystemClock_Config(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

void cpu_init(void)
{
	SystemClock_Config();
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