#include <vehicle_state_machine.h>

volatile double throttle_command_ramp_variable;

double get_throttle_command_ramp_variable(void)
{
	return throttle_command_ramp_variable;
}

void initialize_vehicle_state_machine(vehicle_state *st)
{
	*st = STATE_READY_FOR_TAKEOFF;
	throttle_command_ramp_variable = 0.0f;
}

void propagate_vehicle_state_machine(vehicle_state *st, rc_joystick_data_struct js, filtered_quadrotor_state vehicle_state_vector)
{
	switch(*st)
	{
		/*
			Initialization states:
		 */
		case STATE_IMU_CAL:
			break;
		case STATE_READY_FOR_TAKEOFF:
			/*
				Check if user height request is high enough to arm system for motor ramp-up and go into
				closed-loop height control state:
			 */
			if((float)MAX_HEIGHT*(((js.vertical_channel_value)*0.5f)+0.5f) >= 0.100f)
			{
				throttle_command_ramp_variable = 0.0f;
				*st = STATE_MOTOR_RAMP_UP;
			}
			break;
		case STATE_MOTOR_RAMP_UP:
			#ifdef OPENLOOP_HEIGHT_CONTROL
				if(throttle_command_ramp_variable < MOTOR_RAMP_UP_TARGET_OPENLOOP_HEIGHT)
				{
					throttle_command_ramp_variable += 0.0001f; // 5-second ramp-up when called at 500 Hz
				}
				else
				{
					throttle_command_ramp_variable = MOTOR_RAMP_UP_TARGET_OPENLOOP_HEIGHT;
					*st = STATE_CLOSEDLOOP_ATTITUDE;
				}
			#endif

			#ifdef CLOSEDLOOP_HEIGHT_CONTROL
				if(throttle_command_ramp_variable < MOTOR_RAMP_UP_TARGET_CLOSEDLOOP_HEIGHT)
				{
					throttle_command_ramp_variable += 0.00008f; // 5-second ramp-up when called at 500 kHz
				}
				else
				{
					throttle_command_ramp_variable = MOTOR_RAMP_UP_TARGET_CLOSEDLOOP_HEIGHT;
					*st = STATE_CLOSEDLOOP_ATTITUDE_HEIGHT;
				}
			#endif
			break;

		/*
			Flight operational states:
		 */

		/*
			Height sensor enabled:
		 */
		case STATE_CLOSEDLOOP_LATERAL_POSITION_HEIGHT:
			telemetry_signal_verification(st, js);
			break;
		case STATE_CLOSEDLOOP_LATERAL_VELOCITY_HEIGHT:
			telemetry_signal_verification(st, js);
			break;
		case STATE_CLOSEDLOOP_ATTITUDE_HEIGHT:
			telemetry_signal_verification(st, js);
			break;
		case STATE_CLOSEDLOOP_ATTITUDE_RATE_HEIGHT:
			telemetry_signal_verification(st, js);
			break;

		/*
			Height openloop:
		 */
		case STATE_CLOSEDLOOP_LATERAL_POSITION:
			telemetry_signal_verification(st, js);
			break;
		case STATE_CLOSEDLOOP_LATERAL_VELOCITY:
			telemetry_signal_verification(st, js);
			break;
		case STATE_CLOSEDLOOP_ATTITUDE:
			telemetry_signal_verification(st, js);
			break;
		case STATE_CLOSEDLOOP_ATTITUDE_RATE:
			telemetry_signal_verification(st, js);
			break;

		/*
			Landing modes:
		 */
		case STATE_OPENLOOP_LANDING_ATTITUDE_CONTROL:
			telemetry_signal_verification(st, js);
			break;

		/*
			Error states:
		 */
		case STATE_DC_LINK_VOLTAGE_LOW:
			break;
		case STATE_MOTOR1_LOSS_OF_POWER:
			break;
		case STATE_MOTOR2_LOSS_OF_POWER:
			break;
		case STATE_MOTOR3_LOSS_OF_POWER:
			break;
		case STATE_MOTOR4_LOSS_OF_POWER:
			break;
		case STATE_TELEMETRY_SIGNAL_LOST:
			break;
	}
}

void telemetry_signal_verification(vehicle_state *st, rc_joystick_data_struct js)
{
	if(js.vertical_channel_validity == CHANNEL_INVALID || js.roll_channel_validity == CHANNEL_INVALID ||
		js.pitch_channel_validity == CHANNEL_INVALID || js.yaw_channel_validity == CHANNEL_INVALID)
	{
		*st = STATE_TELEMETRY_SIGNAL_LOST;
	}
}