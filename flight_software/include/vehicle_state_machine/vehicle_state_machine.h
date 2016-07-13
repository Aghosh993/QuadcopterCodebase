#ifndef VEHICLE_STATE_MACHINE
#define VEHICLE_STATE_MACHINE	1

#include "comp_filter.h"
#include "pwm_input.h"
#include "pid_controller.h"

#define MOTOR_RAMP_UP_TARGET_OPENLOOP_HEIGHT	0.2500f
#define MOTOR_RAMP_UP_TARGET_CLOSEDLOOP_HEIGHT	OPENLOOP_MOTOR_MIN_THRUST

typedef enum {
	/*
		Initialization states:
	 */
	STATE_IMU_CAL,
	STATE_READY_FOR_TAKEOFF,
	STATE_MOTOR_RAMP_UP,

	/*
		Flight operational states:
	 */

	/*
		Height sensor enabled:
	 */
	STATE_CLOSEDLOOP_LATERAL_POSITION_HEIGHT,
	STATE_CLOSEDLOOP_LATERAL_VELOCITY_HEIGHT,
	STATE_CLOSEDLOOP_ATTITUDE_HEIGHT,
	STATE_CLOSEDLOOP_ATTITUDE_RATE_HEIGHT,

	/*
		Height openloop:
	 */
	STATE_CLOSEDLOOP_LATERAL_POSITION,
	STATE_CLOSEDLOOP_LATERAL_VELOCITY,
	STATE_CLOSEDLOOP_ATTITUDE,
	STATE_CLOSEDLOOP_ATTITUDE_RATE,

	/*
		Landing modes:
	 */
	STATE_OPENLOOP_LANDING_ATTITUDE_CONTROL,

	/*
		Error states:
	 */
	STATE_DC_LINK_VOLTAGE_LOW,
	STATE_MOTOR1_LOSS_OF_POWER,
	STATE_MOTOR2_LOSS_OF_POWER,
	STATE_MOTOR3_LOSS_OF_POWER,
	STATE_MOTOR4_LOSS_OF_POWER,
	STATE_TELEMETRY_SIGNAL_LOST
} vehicle_state;

void initialize_vehicle_state_machine(vehicle_state *st);
void propagate_vehicle_state_machine(vehicle_state *st, rc_joystick_data_struct js, filtered_quadrotor_state vehicle_state_vector);

void telemetry_signal_verification(vehicle_state *st, rc_joystick_data_struct js);
double get_throttle_command_ramp_variable(void);

#endif
