/*
	File: serial_comms_highlevel_hal.h

	(c) Abhimanyu Ghosh, 2016
 */

#ifndef SERIAL_COMMS_HIGHLEVEL_HAL_H_
#define SERIAL_COMMS_HIGHLEVEL_HAL_H_	1

#include <stdbool.h>
#include <stdint.h>
#include "hal_common_includes.h"
#include "interrupt_utils.h"

typedef enum {
	UART5,
	UART6,
	UART7,

	CAN_ENCAP_UART5,
	CAN_ENCAP_UART6,
	CAN_ENCAP_UART7
} serialport_desc;

/*
	Initializes all required high-level real/virtual serial port HAL drivers:
 */
void serialport_hal_init(void);

void serialport_hal_enable_tx_isr(serialport_desc port_descriptor);
void serialport_hal_disable_tx_isr(serialport_desc port_descriptor);

int serialport_send_byte(serialport_desc port_descriptor, uint8_t byte_to_send);
uint8_t serialport_receive_byte(serialport_desc port_descriptor);

#endif