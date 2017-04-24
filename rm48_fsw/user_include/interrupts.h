/*
	File: interrupts.h

	(c) Abhimanyu Ghosh, 2016
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_	1

// #define SCI1_LOOPBACK	1
// #define SCI2_LOOPBACK	1

#include "hal_common_includes.h"
#include "serial_comms_highlevel.h"
#include "mission_timekeeper.h"

void rti_callback(void);
void sci1_tx_callback(void);
void sci1_rx_callback(void);
void sci2_tx_callback(void);
void sci2_rx_callback(void);
void can_tm4c_port1_rx_message_callback(void);
void can_tm4c_port2_rx_message_callback(void);
void can_tm4c_port3_rx_message_callback(void);
void pwm_callback(hetBASE_t* hetREG, uint32 pwm);

#endif