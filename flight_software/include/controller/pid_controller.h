/*
 * File: pid_controller.h
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2015
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <stdint.h>

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

// #define CLOSEDLOOP_HEIGHT_CONTROL 		1
#define OPENLOOP_HEIGHT_CONTROL 			1

#define YAW_RATE_CLOSED_LOOP_ENABLED		1

//#define PROP_PITCH_4DOT5_DEGREES			1
#define PROP_PITCH_3DOT8_DEGREES			1

#define ATTITUDE_CONTROL_USE_LPF	1

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

#define ROLL_MAX_ABS_ADJUSTMENT			2.00f // Unit: Radians/second
#define PITCH_MAX_ABS_ADJUSTMENT		2.00f // Unit: Radians/second

#define YAW_DEADBAND 0.055f

#define MOTOR_MIN_CMD 0.0f
#define MOTOR_MAX_CMD 1.0f

#define ANGULAR_RATE_CONTROL_DT			0.003f						// About 333 Hz
	#define ROLL_RATE_PID_DT 			ANGULAR_RATE_CONTROL_DT
	#define PITCH_RATE_PID_DT 			ANGULAR_RATE_CONTROL_DT
	#define YAW_RATE_PID_DT				ANGULAR_RATE_CONTROL_DT

#define ANGULAR_POSITION_CONTROL_DT		0.005f 						// Exactly 200 Hz
	#define ROLL_PID_DT					ANGULAR_POSITION_CONTROL_DT
	#define PITCH_PID_DT 				ANGULAR_POSITION_CONTROL_DT
	#define YAW_PID_DT 					ANGULAR_POSITION_CONTROL_DT

#define MAX_HEIGHT			0.500f

/*
	Gains for attitude rate and attitude controllers:
 */
#define roll_rate_kP	0.25f
#define pitch_rate_kP	0.25f

#define yaw_rate_kP		0.20f

#define roll_rate_kI	0.20f //0.2f//0.240f //.08f
#define pitch_rate_kI	0.20f //0.2f//0.240f //.08f
#define yaw_rate_kI		0.20f

#define roll_rate_kD	0.0f
#define pitch_rate_kD	0.0f
#define yaw_rate_kD		0.0f

// Last testing result: Seems to produce highly laggy step response... ????
#define roll_kP 		1.40f //2.8000f
#define pitch_kP 		1.40f //2.8000f
// #define yaw_kP

#define roll_kI 		0.0f//0.12f //0.2050f
#define pitch_kI 		0.0f//0.12f //0.2050f
// #define yaw_kI

#define roll_kD 		0.0f //0.010f
#define pitch_kD 		0.0f //0.010f
// #define yaw_kD

/*
 * Constant feed-forward term to achieve approximately enough thrust on
 * all 4 motors/propellers to generate lift equal to anticipated vehicle
 * weight:
 */
#define OPENLOOP_MOTOR_MIN_THRUST	0.50f

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

void generate_rate_commands(filtered_quadrotor_state* ap_st, float joy_x, float joy_y, float joy_yaw, float* roll_rate_output, float* pitch_rate_output, float* yaw_rate_output);
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

#endif /* PID_CONTROLLER_H_ */
