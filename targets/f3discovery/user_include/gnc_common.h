#ifndef GNC_COMMON_H_
#define GNC_COMMON_H_	1

#include <math.h>
#include <stdint.h>

/*
	Conventions: 
		+x: Forward	
		+y: Right
		+z: Down

		Roll to the RIGHT, Pitch UP (Nose up) and Turning rightware induce a positive change in Roll, Pitch and Yaw respectively
 */

typedef struct {
	float roll;
	float pitch;
	float yaw;

	float roll_rate;
	float pitch_rate;
	float yaw_rate;

	float x;
	float y;
	float z;

	float v_x;
	float v_y;
	float v_z;

	float a_x;
	float a_y;
	float a_z;	
} state_estimate;

typedef struct {
	double gps_lat;
	double gps_long;
	float gps_speed;
	float gps_course;

	float optical_flow_v_x;
	float optical_flow_v_y;

	float sonar_height;
	float lidar_height;
	float pressure_pa;
} observation;

#define degrees_to_radians(deg) 		(deg * M_PI / 180.0f)
#define radians_to_degrees(rad)			(rad * 180.0 / M_PI)

#endif