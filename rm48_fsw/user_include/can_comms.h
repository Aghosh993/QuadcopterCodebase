#ifndef CAN_COMMS_H_
#define CAN_COMMS_H_	1

#include <stdint.h>

typedef enum {
    HEIGHT_HEADING_MSG,
    FLOW_MSG
} sensor_msg;

void get_sensor_msg(uint8_t* msg_data, sensor_msg m);
float get_last_can_height_msg(void);
float get_last_can_heading_msg(void);

#endif