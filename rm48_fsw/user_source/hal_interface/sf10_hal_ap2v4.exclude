/*
 * sf10_hal.c
 *
 *  Created on: Jul 3, 2015
 *      Author: aghosh01
 */

#include "sf10_hal_ap2v4.h"

extern serialport tm4c_port2;
extern serialport *tm4c_port2_ptr;

#define SF10_COMM_PORT      tm4c_port2_ptr

void sf10_hal_setup_ap2v4_serial(void)
{
    serialport_init(SF10_COMM_PORT, TM4C_PORT2); 
}

void send_byte_to_sf10A(uint8_t send_byte)
{
    serialport_send_data_buffer(SF10_COMM_PORT, &send_byte, 1U);
}
