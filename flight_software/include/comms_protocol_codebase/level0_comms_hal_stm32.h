/*
 * level0_comms_hal_stm32
 *
 *  Created on: Feb 28, 2016
 *      Author: aghosh01
 */

#ifndef LEVEL0_COMMS_HAL_STM32_H_
#define LEVEL0_COMMS_HAL_STM32_H_

#include <stdint.h>
#include <hal_common_includes.h>

#define DISABLE_INTERRUPTS _disable_interrupts()
#define ENABLE_INTERRUPTS _enable_interrupts()

void send_byte_to_gs(uint8_t send_byte);

#endif /* LEVEL0_COMMS_HAL_STM32_H_ */
