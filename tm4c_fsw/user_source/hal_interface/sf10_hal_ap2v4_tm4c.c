/*
 * sf10_hal.c
 *
 *  Created on: Jul 3, 2015
 *      Author: aghosh01
 */

#include "sf10_hal_ap2v4_tm4c.h"

extern serialport uart6_port;

void sf10_hal_setup_ap2v4_tm4c_serial(void)
{
    serialport_init(&uart6_port, UART6); 
}

void send_byte_to_sf10A(uint8_t send_byte)
{
    serialport_send_data_buffer(&uart6_port, &send_byte, 1U);
}
