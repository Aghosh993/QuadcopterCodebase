/*
 * File: pid_controller.h
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2015
 */

#ifndef VEHICLE_PID_CONTROLLER_H_
#define VEHICLE_PID_CONTROLLER_H_	1

#include <stdint.h>

#include "basic_pid_controller.h"
#include "iir_filters.h"
#include "comp_filter.h"

/*
 * Comment in/out the following lines to
 * enable/disable
 */

#define ROLL_AXIS_PID_ENA 1
#define PITCH_AXIS_PID_ENA 1

#define MOTOR_1	0
#define MOTOR_2	1
#define MOTOR_3 2
#define MOTOR_4	3

#ifdef ROLL_AXIS_PID_ENA
	#define MOTOR_3_EN 1
	#define MOTOR_4_EN 1
#endif

#ifdef PITCH_AXIS_PID_ENA
	#define MOTOR_1_EN 1
	#define MOTOR_2_EN 1
#endif

#define SINGLE_STAGE_HEIGHT_CONTROLLER 		1
// #define NESTED_HEIGHT_CONTROLLER 			1

#define CLOSEDLOOP_HEIGHT_CONTROL 		1
// #define OPENLOOP_HEIGHT_CONTROL 			1

#define YAW_RATE_CLOSED_LOOP_ENABLED		1

//#define PROP_PITCH_4DOT5_DEGREES			1
#define PROP_PITCH_3DOT8_DEGREES			1

// #define ATTITUDE_CONTROL_USE_LPF	1

#ifdef ATTITUDE_CONTROL_USE_LPF
 #define ROLL_PID_DERIV_LPF_FREQ	15.0f
 #define PITCH_PID_DERIV_LPF_FREQ	15.0f
#endif

#define HALF_SQRT_2 					0.707106781f

#define ROLL_RATE_ACCUM_ERR_ABS_MAX 	0.30f // Units: Radians
#define ROLL_RATE_MAX_ABS_ADJUSTMENT 	0.30f

#define PITCH_RATE_ACCUM_ERR_ABS_MAX 	0.30f // Units: Radians
#define PITCH_RATE_MAX_ABS_ADJUSTMENT 	0.30f

#define YAW_RATE_ACCUM_ERR_ABS_MAX 		0.30f // Units: Radians
#define YAW_RATE_MAX_ABS_ADJUSTMENT 	0.30f

#define ROLL_ACCUM_ERR_ABS_MAX 			0.30f // Units: Radian*seconds
#define PITCH_ACCUM_ERR_ABS_MAX 		0.30f // Units: Radian*seconds

#define ROLL_MAX_ABS_ADJUSTMENT			4.00f // Unit: Radians/second
#define PITCH_MAX_ABS_ADJUSTMENT		4.00f // Unit: Radians/second

#define YAW_DEADBAND 0.055f

#define MOTOR_MIN_CMD 0.0f
#define MOTOR_MAX_CMD 1.0f

#define ANGULAR_RATE_CONTROL_DT			0.004f						// 250 Hz
	#define ROLL_RATE_PID_DT 			ANGULAR_RATE_CONTROL_DT
	#define PITCH_RATE_PID_DT 			ANGULAR_RATE_CONTROL_DT
	#define YAW_RATE_PID_DT				ANGULAR_RATE_CONTROL_DT

#define ANGULAR_POSITION_CONTROL_DT		0.005f						// 125 Hz
	#define ROLL_PID_DT					ANGULAR_POSITION_CONTROL_DT
	#define PITCH_PID_DT 				ANGULAR_POSITION_CONTROL_DT
	#define YAW_PID_DT 					ANGULAR_POSITION_CONTROL_DT

#define LATERAL_VELOCITY_CONTROL_DT		0.020f
 
#define HEIGHT_CONTROL_DT				0.006f

#define MAX_HEIGHT						0.500f	// In Meters

/*
	Gains for attitude rate and attitude controllers:
 */
#define roll_rate_kP	0.20f
#define pitch_rate_kP	0.20f
#define yaw_rate_kP		0.20f

#define roll_rate_kI	0.20f
#define pitch_rate_kI	0.20f
#define yaw_rate_kI		0.20f

#define roll_rate_kD	0.0f
#define pitch_rate_kD	0.0f
#define yaw_rate_kD		0.0f

#define roll_kP 		1.70f
#define pitch_kP 		1.70f

#define roll_kI 		0.0f
#define pitch_kI 		0.0f

#define roll_kD 		0.005f
#define pitch_kD 		0.005f


#define GND_EFFECT_TRANSITION_POINT1		0.1f
 #define GND_EFFECT_TRANSITION_POINT1_BIAS	0.08f
#define GND_EFFECT_TRANSITION_POINT2		0.2f
 #define GND_EFFECT_TRANSITION_POINT2_BIAS	0.04f

/*
 * Constant feed-forward term to achieve approximately enough thrust on
 * all 4 motors/propellers to generate lift equal to anticipated vehicle
 * weight:
 */
#define OPENLOOP_MOTOR_MIN_THRUST			0.50f

/*
 * Constant feed-forward term to achieve approximately enough thrust on
 * 3 motors/propellers to generate lift equal to anticipated vehicle
 * weight:
 */
#define OPENLOOP_MOTOR_MIN_THRUST_THREE_MOTORS	0.350f

typedef enum {
	MODE_ANGULAR_RATE_CONTROL,
	MODE_ANGULAR_POSITION_CONTROL,
	MODE_XY_HEIGHT_POSITION_FEEDBACK
} controller_mode;

/*
 * Function Prototypes:
 */

/*
 * Initialization:
 */

void controller_init_vars(void);

/*
 * Moding/functionality control:
 */

void enable_controller(void);
void disable_controller(void);
uint8_t get_controller_status(void);
void set_controller_mode(controller_mode mode);

/*
 * Actuation saturation verification/control:
 */

void check_output_saturation(double* motor_output_buffer);

/*
 * Low-level attitude rate inner loop major cycle update function,
 * and outer-loop function to generate rates for inner loop based
 * on user/code-driven high-level input of desired vehicle attitude:
 */

void generate_rate_commands(filtered_quadrotor_state* ap_st, imu_scaled_data_struct* data, float joy_x, float joy_y, float joy_yaw, float* roll_rate_output, float* pitch_rate_output, float* yaw_rate_output);
void rate_controller_update(double * c_props, imu_scaled_data_struct* data, float roll_rate_cmd, float pitch_rate_cmd, float yaw_rate_cmd, float throttle_openloop_commanded);

/*
 * High-level outer-loop functions to generate vehicle desired attitude
 * and throttle commands based on parameters such as lateral X and Y
 * velocity, vertical velocity and height over ground:
 */
#ifdef NESTED_HEIGHT_CONTROLLER
void generate_vertical_acceleration_commands(float actual_height, float target_height, float* vertical_acceleration_output);
void generate_thrust_commands(float actual_vertical_acceleration, float target_vertical_acceleration, float* thrust_output);
#endif

#ifdef SINGLE_STAGE_HEIGHT_CONTROLLER
void generate_thrust_commands(float actual_height, float target_height, float* throttle_command);
#endif

void generate_attitude_commands(float velocity_x, float velocity_y, float target_velocity_x, float target_velocity_y, float* roll_output, float* pitch_output);

void rate_controller_integral_disable();
void attitude_controller_integral_disable();

void rate_controller_integral_enable();
void attitude_controller_integral_enable();
#endif /* VEHICLE_PID_CONTROLLER_H_ */
