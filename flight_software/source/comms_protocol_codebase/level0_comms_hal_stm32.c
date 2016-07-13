/*
 * level0_comms_hal_rm48.c
 *
 *  Created on: Jul 15, 2015
 *      Author: aghosh01
 */

#include "level0_comms_hal_stm32.h"

void send_byte_to_gs(uint8_t send_byte)
{
	usart_send_blocking(USART2, send_byte);
}
