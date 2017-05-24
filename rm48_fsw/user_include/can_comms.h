#ifndef CAN_COMMS_H_
#define CAN_COMMS_H_	1

#include <stdint.h>
#include "hal_common_includes.h"

typedef enum {
    HEIGHT_HEADING_MSG,
    FLOW_MSG,
    BNO_ATT_MSG
} sensor_msg;

void get_sensor_msg(uint8_t* msg_data, sensor_msg m);
float get_last_can_height_msg(void);
float get_last_can_heading_msg(void);
void get_last_attitude(float *att);

void publish_roll_pitch(float roll, float pitch);
void publish_yaw_height_estimate(float yaw, float height_estimate);
void publish_vert_velocity_estimate(float v);
void publish_motor_commands(float *cmds);
int new_height_avail(void);

#endif