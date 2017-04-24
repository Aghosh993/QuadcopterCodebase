/*
 * @File: 	basic_pid_controller.h
 * @Author: Abhimanyu Ghosh
 * @Date: 	10/02/2015
  
 * @Brief: 			General-purpose PID (Proportional-Integral-Derivative) controller library header.
 * @Description: 	Implements the basic functions required for the initialization, control and update of generic
 					    PID controllers
 * @Licensing: This codebase shall be considered open and freely-shareable software under general usage circumstances, and
 			       is thus considered to be covered under a BSD open-source license.
 */

#ifndef BASIC_PID_CONTROLLER_H_
#define BASIC_PID_CONTROLLER_H_

#include <stdint.h>

#include "iir_filters.h"

#define PID_STRUCT_SIZE_BYTES		34U
#define PI_D_LPF_STRUCT_SIZE_BYTES	42U

#define DERIVATIVE_ON_MEASUREMENT	1

typedef enum {
	PID_CHECKSUM_PASS,
	PID_CHECKSUM_FAIL
} pid_checksum_result;

typedef enum {
	CONTROLLER_ENABLED,
	CONTROLLER_DISABLED
} pid_controller_enable_state;

typedef struct {

	#ifndef DERIVATIVE_ON_MEASUREMENT
		float last_error_value;
		float last_commanded_state;
	#endif
	#ifdef DERIVATIVE_ON_MEASUREMENT
		float last_state_measurement;
	#endif

	float accumulated_error;
	float error_saturation_absolute_value;
	float maximum_pid_adjustment_value;
	float dt;								// This is in seconds
	float k_p;
	float k_i;
	float k_d;

	uint8_t integral_en;
	uint8_t derivative_en;

	pid_controller_enable_state control_enable_state;
	uint8_t pid_struct_checksum;
} pid_data_struct;

/*
	Functions for basic PID controller:
 */

void init_pid_controller(pid_data_struct* str, float k_p, float k_i, float k_d,
							float dt, float integral_windup_max_abs_val,
							float pid_max_adjustment_abs_val);
void pid_set_kp(pid_data_struct* str, float kp);
void pid_set_ki(pid_data_struct* str, float ki);
void pid_set_kd(pid_data_struct* str, float kd);

void pid_inhibit_integral(pid_data_struct* str);
void pid_enable_integral(pid_data_struct* str);

void pid_inhibit_derivative(pid_data_struct* str);
void pid_enable_derivative(pid_data_struct* str);

void enable_pid_controller(pid_data_struct* str);
void disable_pid_controller(pid_data_struct* str);
float calculate_pid_adjustment(pid_data_struct* str, float state_measurement, float commanded_state);
float calculate_pid_adjustment_derivative_passthrough(pid_data_struct* str, float state_measurement, float derivative_state_measurement, float commanded_state);
float calculate_pid_adjustment_integral_passthrough(pid_data_struct* str, float state_measurement, float integral_state_measurement, float commanded_state);
void copy_pid_struct_data(pid_data_struct* dest, const pid_data_struct src);
uint8_t calc_pid_struct_checksum(pid_data_struct str);
pid_checksum_result verify_pid_struct_checksum(const pid_data_struct str);

/*
	Functions for PID with a first-order IIR low-pass filter on the derivative term:
 */

typedef struct {

	#ifndef DERIVATIVE_ON_MEASUREMENT
		float last_error_value;
		float last_commanded_state;
	#endif
	#ifdef DERIVATIVE_ON_MEASUREMENT
		float last_state_measurement;
	#endif

	float accumulated_error;
	float error_saturation_absolute_value;
	float maximum_pid_adjustment_value;
	float dt;								// This is in seconds
	float k_p;
	float k_i;
	float k_d;

	uint8_t integral_en;
	uint8_t derivative_en;
	
	lpf_data_struct derivative_lpf_struct;

	pid_controller_enable_state control_enable_state;
	uint8_t pid_struct_checksum;
} pi_d_lpf_data_struct;

void init_pi_d_lpf_controller(pi_d_lpf_data_struct* str, float k_p, float k_i, float k_d,
									float dt, float integral_windup_max_abs_val,
									float pid_max_adjustment_abs_val,
									float derivative_lpf_cutoff_freq_hertz);
void pi_d_lpf_set_kp(pi_d_lpf_data_struct* str, float kp);
void pi_d_lpf_set_ki(pi_d_lpf_data_struct* str, float ki);
void pi_d_lpf_set_kd(pi_d_lpf_data_struct* str, float kd);

void pi_d_lpf_inhibit_integral(pi_d_lpf_data_struct* str);
void pi_d_lpf_enable_integral(pi_d_lpf_data_struct* str);

void pi_d_lpf_inhibit_derivative(pi_d_lpf_data_struct* str);
void pi_d_lpf_enable_derivative(pi_d_lpf_data_struct* str);

void enable_pi_d_lpf_controller(pi_d_lpf_data_struct* str);
void disable_pi_d_lpf_controller(pi_d_lpf_data_struct* str);
float calculate_pi_d_lpf_adjustment(pi_d_lpf_data_struct* str, float state_measurement, float commanded_state);
float calculate_pi_d_lpf_adjustment_derivative_passthrough(pi_d_lpf_data_struct* str, float state_measurement, float derivative_state_measurement, float commanded_state);
float calculate_pi_d_lpf_adjustment_integral_passthrough(pi_d_lpf_data_struct* str, float state_measurement, float integral_state_measurement, float commanded_state);
void copy_pi_d_lpf_struct_data(pi_d_lpf_data_struct* dest, const pi_d_lpf_data_struct src);
uint8_t calc_pi_d_lpf_struct_checksum(pi_d_lpf_data_struct str);
pid_checksum_result verify_pi_d_lpf_struct_checksum(const pi_d_lpf_data_struct str);

/* Not fully implemented yet...: */

// typedef struct {
// 	uint32_t num_elements;
// 	float* error_array;
// 	float* actuation_commands_array;
// } pid_ff_data_struct;

// int init_pid_feedforward_data_struct(pid_ff_data_struct* str, uint32_t n_data_points, 
// 										float* error_array_input, float* actuation_commands_array_input);

#endif /* BASIC_PID_CONTROLLER_H_ */
