#ifndef POLLING_DELAY_H
#define POLLING_DELAY_H 1

// Delay function prototype and define(s):

/*
 * Delay cycles per millisecond parameter
 * Depends on system clock... adjust and test with oscilloscope as appropriate:
 */

#define CPU_CYCLES_PER_MS 32000

/*
 * Delay function prototype:
 */
void insert_delay_ms(int ms);

#endif
