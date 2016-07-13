#include <interrupts.h>

#include <stdint.h>
#include <mission_timekeeper.h>

#include <pwm_input_hal.h>

/*
At the moment, libopencm3 appears not to have macros/functions
explicitly defined to globally disable/enable interrupts.
Thus, the following two wrapper functions for some assembler are
required to achieve this.

Taken from:
http://permalink.gmane.org/gmane.comp.lib.libopencm3/29
 */

/*
Globally ENABLE interrupts:
 */

inline void _enable_interrupts(void)
{
	asm volatile ("cpsie i");
}

/*
Globally DISABLE interrupts:
 */

inline void _disable_interrupts(void)
{
	asm volatile ("cpsid i");
}

/*
Set up 1 kHz Systick interrupt as system-wide time base.

Adapted from: 
https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/obldc/systick/systick.c
 */

void systick_setup(void)
{
	/*
		64 MHz SYSCLK /8 = 8 MHz Systick clock
	 */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/*
		Overflow at 8 MHz / (7999+1U) = 1 kHz:
	 */
	systick_set_reload(7999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

/*
Overrides the WEAK declaration of systick_tick_handler() in nvic.h:
 */

void sys_tick_handler(void)
{
	flag_scheduler_callback();
	update_mission_time_counter();
	rc_input_watchdog_callback();
}

void tim2_isr(void)
{
	timer2_isr_callback();
}

void tim3_isr(void)
{
	timer3_isr_callback();
}

void tim4_isr(void)
{
	timer4_isr_callback();
}

void tim8_cc_isr(void)
{
	timer8_isr_callback();
}