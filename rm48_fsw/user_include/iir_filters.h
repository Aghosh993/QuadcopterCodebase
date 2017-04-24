/*
 * File: iir_filters.h
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2016
 */

#ifndef IIR_FILTER_H_
#define IIR_FILTER_H_

#include <math.h>

#define PI_VAL			3.14159f

#define RADIANS_TO_DEGREES_CONVERSION_FACTOR	((float)180.0/(float)PI_VAL)
#define DEGREES_TO_RADIANS_CONVERSION_FACTOR	((float)PI_VAL/(float)180.0)

#define DT_FILTER_LOOP	0.001f // Interval at which the filter is run, in seconds

// LPF-related data structure(s):

typedef struct
{
	float prev_output;
	float alpha_val;
} lpf_data_struct;

// LPF function prototypes:
void init_lpf_variables(lpf_data_struct* lpf_str, float cutoff_freq_hertz);
float lowpass_filter(float input, lpf_data_struct* lpf_str);

// LPF-related data structure(s):

typedef struct
{
	float prev_input;
	float prev_output;
	float alpha_val;
} hpf_data_struct;

// HPF function prototypes:
void init_hpf_variables(hpf_data_struct* hpf_str, float cutoff_freq_hertz);
float highpass_filter(float input, hpf_data_struct* hpf_str);

#endif /* IIR_FILTER_H_ */
