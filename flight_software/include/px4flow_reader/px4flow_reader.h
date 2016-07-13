/*
 * pxflow_reader.h
 *
 *  Created on: Jan 22, 2015
 *      Author: aghosh01
 */

#ifndef PXFLOW_READER_H_
#define PXFLOW_READER_H_

#include "mavlink.h"

typedef unsigned char uint8;

typedef struct {
	float x_velocity;
	float y_velocity;
} pxflow_flow_data_struct;

float get_pxflow_height(void);
void get_pxflow_flow_data(pxflow_flow_data_struct *buffer);

void px4flow_interrupt_callback(uint8 rxbyte);

#endif /* PXFLOW_READER_H_ */
