/*
 * @File: 	basic_pid_controller.h
 * @Author: Abhimanyu Ghosh
 * @Date: 	10/02/2015
  
 * @Brief: 			General-purpose PID (Proportional-Integral-Derivative) controller library header.
 * @Description: 	Implements the basic functions required for the initialization, control and update of generic
 					    PID controllers
 * @Licensing: This codebase shall be considered open and freely-shareable software under general usage circumstances, and
 			       is thus considered to be covered under a BSD open-source license.
 			   The primary exception to the aforementioned rule is in the context of usage of this software in systems covered
 			       under U.S. Export Control Law, and listed under the USML (United States Munitions List) and prohibited
 			   	   from exports (deemed or otherwise). "Exports" in this context include, but are not limited to disclosure
 			   	   of key technical details and/or source code to foreign nationals or non-"U.S. Persons" as defined in 22 U.S.C. 2778.
 			   Further details may be obtained by legal analysis of ITAR regulations, and consultation with necessary legal counsel 
 			       and acquisition of an export license through written agreement with the U.S. State Department.
 			   None of the information contained above is to be considered legal advice, and the stated author of this source code
 			       shall not under any circumstance be held liable for penalties arising from violations of laws listed above, or others.
 */

#ifndef BASIC_PID_CONTROLLER_H_
#define BASIC_PID_CONTROLLER_H_

#include <stdint.h>

#include "comp_filter.h"

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
	pid_controller_enable_state control_enable_state;
	uint8_t pid_struct_checksum;
} pid_data_struct;

typedef struct {

	#ifndef DERIVATIVE_ON_MEASUREMENT
		float last_error_value;
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

	lpf_data_struct derivative_lpf_struct;

	pid_controller_enable_state control_enable_state;
	uint8_t pid_struct_checksum;
} pi_d_lpf_data_struct;

typedef struct {
	uint32_t num_elements;
	float* error_array;
	float* actuation_commands_array;
} pid_ff_data_struct;

void init_pid_controller(pid_data_struct* str, float k_p, float k_i, float k_d,
							float dt, float integral_windup_max_abs_val,
							float pid_max_adjustment_abs_val);
void enable_pid_controller(pid_data_struct* str);
void disable_pid_controller(pid_data_struct* str);
float calculate_pid_adjustment(pid_data_struct* str, float state_measurement, float commanded_state);
void copy_pid_struct_data(pid_data_struct* dest, const pid_data_struct src);
uint8_t calc_pid_struct_checksum(pid_data_struct str);
pid_checksum_result verify_pid_struct_checksum(const pid_data_struct str);

void init_pi_d_lpf_controller(pi_d_lpf_data_struct* str, float k_p, float k_i, float k_d,
									float dt, float integral_windup_max_abs_val,
									float pid_max_adjustment_abs_val,
									float derivative_lpf_cutoff_freq_hertz);
void enable_pi_d_lpf_controller(pi_d_lpf_data_struct* str);
void disable_pi_d_lpf_controller(pi_d_lpf_data_struct* str);
float calculate_pi_d_lpf_adjustment(pi_d_lpf_data_struct* str, float state_measurement, float commanded_state);
void copy_pi_d_lpf_struct_data(pi_d_lpf_data_struct* dest, const pi_d_lpf_data_struct src);
uint8_t calc_pi_d_lpf_struct_checksum(pi_d_lpf_data_struct str);
pid_checksum_result verify_pi_d_lpf_struct_checksum(const pi_d_lpf_data_struct str);

int init_pid_feedforward_data_struct(pid_ff_data_struct* str, uint32_t n_data_points, 
										float* error_array_input, float* actuation_commands_array_input);

#endif /* BASIC_PID_CONTROLLER_H_ */
