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

void publish_roll_pitch(float roll, float pitch)
{
	union {
		float input[2];
		uint8_t output[8];
	} conv_f32_2_to_u8;

	conv_f32_2_to_u8.input[0] = roll;
	conv_f32_2_to_u8.input[1] = pitch;

	canTransmit(canREG3, canMESSAGE_BOX3, conv_f32_2_to_u8.output);
}

void publish_yaw_height_estimate(float yaw, float height_estimate)
{
	union {
		float input[2];
		uint8_t output[8];
	} conv_f32_2_to_u8;

	conv_f32_2_to_u8.input[0] = yaw;
	conv_f32_2_to_u8.input[1] = height_estimate;

	canTransmit(canREG3, canMESSAGE_BOX4, conv_f32_2_to_u8.output);	
}

void publish_vert_velocity_estimate(float v)
{
	union {
		float input[2];
		uint8_t output[8];
	} conv_f32_2_to_u8;

	conv_f32_2_to_u8.input[0] = v;
	conv_f32_2_to_u8.input[1] = 0.0f;

	canTransmit(canREG3, canMESSAGE_BOX5, conv_f32_2_to_u8.output);
}