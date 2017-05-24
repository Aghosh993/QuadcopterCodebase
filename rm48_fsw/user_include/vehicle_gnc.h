#ifndef VEHICLE_GNC_H_
#define VEHICLE_GNC_H_		1

#include "imu.h"
#include "iir_filters.h"
#include "complementary_filter.h"
#include "kalman_4state.h"
#include "kalman_3state_height.h"
#include "vehicle_pid_controller.h"

#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#define RAPID_DESCENT_INITIAL_HEIGHT		0.80f 		// In Meters
#define RAPID_DESCENT_VELOCITY				0.65f 		// In Meters*s^-1
#define RAPID_DESCENT_MOTOR_CUTOFF_HEIGHT	0.08f 		// In Meters

#define ATTITUDE_CMD_MULTIPLIER				0.60f

#define ACTUATOR_LOW_LIM					0.01f
#define ACTUATOR_HIGH_LIM					0.97f

typedef enum {
	MODE_RATE_CONTROL, 						// Attitude rate control in all 3 axes, throttle/height is open-loop
	MODE_ATTITUDE_CONTROL, 					// Attitude control in all 3 axes
	MODE_ATTITUDE_HEIGHT_CONTROL,			// Attitude control in all 3 axes + height control
	MODE_ATTITUDE_HEIGHT_HEADING_CONTROL,	// Attitude control in all 3 axes + height + heading control
	MODE_VELOCITY_CONTROL					// Lateral and vertical velocity control and heading control
} gnc_control_mode;

void gnc_init(float attitude_kP, float attitude_kD, float rate_kP, gnc_control_mode m);

void gnc_enable(void);
void gnc_disable(void);
uint8_t gnc_enabled(void);

void gnc_update_vehicle_state(void);
void gnc_integral_disable(void);
void gnc_integral_enable(void);

void gnc_attitude_controller_update(float roll_cmd_in, float pitch_cmd_in, float yaw_rate_cmd_in);
void gnc_attitude_rate_controller_update(float throttle_value_in);
void gnc_get_actuator_commands(float* commands);
float gnc_get_height_controller_throttle_command(float height_commanded);
// float gnc_height_controller_thrust_offset(float rotor_dia_meters, float height_meters);

float gnc_compass_get_relative_heading(float raw_heading, float prior_heading);
// float get_compensated_sf10_data(vehicle_relative_height_tracker *tr, 
// 											float sf10_raw_measurement, float sf10_previous_raw_measurement,
// 											float current_height_cmd, float previous_height_cmd);
// void automated_landing_sequence(float relative_time, float *height_setpoint_output, float actual_height);
// void estimate_lateral_velocity(float* velocity_x_output, float *velocity_y_output, pxflow_flow_data_struct flow_data_input, 
// 								float height_input, float current_roll_degrees, float current_pitch_degrees);
// void px4flow_get_bias(int16_t* bias_x, int16_t* bias_y);

void gnc_get_raw_sensor_data(observation *ret);
void gnc_get_state_vector_data(complementary_filter_struct *ret);

void get_height_state_vector(float* h, float* v, float* a);

#endif