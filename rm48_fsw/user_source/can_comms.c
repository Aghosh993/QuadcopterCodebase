#include "can_comms.h"

static volatile float last_lidar_height;
static volatile float last_heading_reading;

void get_sensor_msg(uint8_t* msg_data, sensor_msg m)
{
    int i = 0;
	
	union {
        float output[2];
        uint8_t input[8];
    } uint8_to_f32_2;

	switch(m)
	{
		case HEIGHT_HEADING_MSG:

		    for(i=0; i<8; ++i)
		    {
		    	uint8_to_f32_2.input[i] = msg_data[i];
		    }

		    last_lidar_height = uint8_to_f32_2.output[0];
		    last_heading_reading = uint8_to_f32_2.output[1];
			break;
		case FLOW_MSG:
			break;
	}
}

float get_last_can_height_msg(void)
{
	return last_lidar_height;
}

float get_last_can_heading_msg(void)
{
	return last_heading_reading;
}