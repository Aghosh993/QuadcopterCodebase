#ifndef VEHICLE_GNC_H_
#define VEHICLE_GNC_H_		1

#include "imu.h"
#include "iir_filters.h"
#include "comp_filter.h"
#include "vehicle_pid_controller.h"

#include <stdint.h>
#include <math.h>

#define SF10_UGV_THRESHOLD	0.18f // in meters, was 0.1f in last flight test
#define UGV_HEIGHT			0.50f // in meters

#define RAPID_DESCENT_INITIAL_HEIGHT		0.80f 		// In Meters
#define RAPID_DESCENT_VELOCITY				0.65f 		// In Meters*s^-1
#define RAPID_DESCENT_MOTOR_CUTOFF_HEIGHT	0.08f 		// In Meters

// #define VELOCITY_CONTROL 	1

typedef enum {
	STATE_VEHICLE_ABOVE_GROUND,
	STATE_TRANSITIONING_TO_UGV,
	STATE_VEHICLE_ABOVE_UGV,
	STATE_TRANSITIONING_TO_GROUND
} vehicle_relative_height_tracker;

typedef struct {
	/*Filter Constants:*/

	/*Time between filter updates in seconds:*/
	float filter_dt;
	/*Noise characteristic of LIDAR Altimeter:*/
	float r_lidar;

	/*========================================*/

	/*Filter Variables*/

	/*Filter state estimate (aka The Money :))*/
	float height_estimated;
	float vertical_velocity_estimated;

	/*Latest sensor data fed into the filter as of the last update cycle:*/
	float h_lidar_global_coords;
	float a_accel_global_coords;

	/*P matrix elements:*/
	float p11;
	float p12;
	float p21;
	float p22;

	/*G (Kalman Gain) matrix elements:*/
	float g11;
	float g21;
} height_kalman_data_struct;

typedef struct {
	float roll_gyro;
	float pitch_gyro;
	float yaw_gyro;

	float x_accel;
	float y_accel;
	float z_accel;

	float height_lidar;

	float x_vel_flow;
	float y_vel_flow;
	float height_flow_sensor;

	float bno055_heading;
} gnc_raw_data;

typedef struct {
	float roll;
	float pitch;
	float yaw;

	float roll_rate;
	float pitch_rate;
	float yaw_rate;

	float height;
	float vertical_velocity;
} gnc_state_data;

void gnc_init(void);

void gnc_enable(void);
void gnc_disable(void);
uint8_t gnc_enabled(void);

void gnc_get_vehicle_state(void);
// void velocity_controller_update(float vel_cmd_x, float vel_cmd_y, float vel_x, float vel_y, float *roll_cmd, float *pitch_cmd);
void gnc_integral_disable(void);
void gnc_integral_enable(void);
void gnc_vehicle_stabilization_outerloop_update(float roll_cmd_in,
											float pitch_cmd_in,
											float yaw_cmd_in,
											float *roll_rate_cmd_out,
											float *pitch_rate_cmd_out,
											float *yaw_rate_cmd_out);
void gnc_vehicle_stabilization_innerloop_update(float roll_rate_cmd_in,
											float pitch_rate_cmd_in,
											float yaw_rate_cmd_in,
											float throttle_value_in,
											double *motor_commands_out);
void gnc_height_kalman_struct_init(height_kalman_data_struct *str, float filter_dt_sec, float r_lidar);
void gnc_height_kalman_update(height_kalman_data_struct *str, float lidar_height_measurement, float accelerometer_z_measurement, float vehicle_roll_deg, float vehicle_pitch_deg);
float gnc_bno055_get_relative_heading(float raw_heading, float prior_heading);
float gnc_height_controller_thrust_offset(float rotor_dia_meters, float height_meters);
float gnc_get_height_controller_throttle_command(float height_commanded, float height_estimate, float vertical_velocity_estimate);
// float get_compensated_sf10_data(vehicle_relative_height_tracker *tr, 
// 											float sf10_raw_measurement, float sf10_previous_raw_measurement,
// 											float current_height_cmd, float previous_height_cmd);
// void automated_landing_sequence(float relative_time, float *height_setpoint_output, float actual_height);
// void estimate_lateral_velocity(float* velocity_x_output, float *velocity_y_output, pxflow_flow_data_struct flow_data_input, 
// 								float height_input, float current_roll_degrees, float current_pitch_degrees);
// void px4flow_get_bias(int16_t* bias_x, int16_t* bias_y);

void gnc_get_raw_sensor_data(gnc_raw_data *ret);
void gnc_get_state_vector_data(gnc_state_data *ret);

float gnc_get_vertical_dynamic_acceleration(void);

#endif