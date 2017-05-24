/*Standard HAL (Hardware Abstraction Layer) includes:*/
#include "hal_common_includes.h"
#include "interrupt_utils.h"

// User/flight software libraries:
// #include "basic_pid_controller.h"
// #include "iir_filters.h"
#include "Quadcopter_PWM_HAL.h"
#include "rt_telemetry.h"
#include "misc_utils.h"
#include "pwm_input.h"
#include "imu.h"
#include "mission_timekeeper.h"
#include "board_led.h"
#include "telem_config.h"
#include "vehicle_gnc.h"
#include "serial_comms_highlevel.h"
#include "cpu_hal_interface.h"
#include "system_shell.h"
#include "can_comms.h"
#include "complementary_filter.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
	Quadcopter primary flight software file.
	This file performs overall system initialization, setup and realtime control of a quadcopter.
	It also handles various operational modes, failsafes and command data I/O.

	All code contained within this file is the original work of the copyright-holder named below, and
	is licensed under the GPLv3 open-source software license. As such, this work is a DERIVED work of an existing GPLv3
	project by the same copyright-holder, and is thus released under an identical license.

	The end user of this file is responsible for observing all applicable patent, intellectual property and export control laws
	in his/her jurisdiction, and releases the person named below from any liabilities, with the understanding that this
	source code is provided "as-is".

	(c) 2016-2017, Abhimanyu Ghosh
 */

/* USER CODE END */

/*
	Some unavoidable global definitions that are also accessed by functions called from interrupt
	handlers in interrupts.c:
 */

volatile serialport ftdi_dbg_port;
volatile rt_telemetry_comm_channel telem0;

volatile float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
volatile float throttle_value_common;

volatile uint8_t gnc_innerloop_flag;

volatile rc_joystick_data_struct rc_data;

volatile uint8_t flag_1hz;
volatile uint8_t flag_10hz;
volatile uint8_t flag_20hz;
volatile uint8_t flag_50hz;
volatile uint8_t flag_100hz;
volatile uint8_t flag_200hz;
volatile uint8_t flag_1000hz;

volatile float m1_cmd, m2_cmd, m3_cmd, m4_cmd;
volatile uint8_t update_cmds;
/*
  Apps defined for the various commands:
 */

void shell_clear(int argc, char** argv)
{
  clear_buffer();
}

void ledcontrol(int argc, char** argv)
{
  led l = LED1;

  if(argc < 2)
  {
    printf("Usage: led_ctl [led1/led2] [on/off]\r\n");
  }
  else
  {
    if(strncmp(argv[0], "led1", 4) == 0)
    {
      l = LED1;
    }
    else
    {
      if(strncmp(argv[0], "led2", 4) == 0)
      {
        l = LED2;
      }
      else
      {
        printf("INVALID LED selected\r\n");
      }
    }
    if(strncmp(argv[1], "on", 3) == 0)
    {
      board_led_on(l);
    }
    if(strncmp(argv[1], "off", 3) == 0)
    {
      board_led_off(l);
    }
  }
}

void motorcontrol(int argc, char** argv)
{
	// Initialize PWM channels for 4 motors and set all motors to stop by default:
	QuadRotor_PWM_init();

	float pwr = 0.0f;

	if(argc < 2)
	{
		printf("Usage: setmotor [1/2/3/4/all] [pwr] with pwr a float from 0-1\r\n");
	}
	else
	{
		if(strncmp(argv[1], "off", 3) == 0)
		{
			pwr = 0.0f;
		}
		else
		{
			sscanf(argv[1], "%f", &pwr);

			if(pwr > 0.0f && pwr < 1.0f)
			{
				printf("\r\nSetting motor value %.2f\r\n", pwr);
			}
			else
			{
				printf("ERROR, power value (arg 2) out of range!! Must be in range [0.0, 1.0]\r\n");
				return;
			}
		}
		
		if(strncmp(argv[0], "1", 1) == 0)
		{
			if(!is_setup(1))
			{
				QuadRotor_motor1_start();
				QuadRotor_motor1_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			QuadRotor_motor1_setDuty(pwr);
		}
		else if(strncmp(argv[0], "2", 1) == 0)
		{
			if(!is_setup(2))
			{
				QuadRotor_motor2_start();
				QuadRotor_motor2_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			QuadRotor_motor2_setDuty(pwr);
		}
		else if(strncmp(argv[0], "3", 1) == 0)
		{
			if(!is_setup(3))
			{
				QuadRotor_motor3_start();
				QuadRotor_motor3_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			QuadRotor_motor3_setDuty(pwr);			
		}
		else if(strncmp(argv[0], "4", 1) == 0)
		{
			if(!is_setup(4))
			{
				QuadRotor_motor4_start();
				QuadRotor_motor4_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			QuadRotor_motor4_setDuty(pwr);			
		}
		else if(strncmp(argv[0], "all", 3) == 0)
		{
			if(!is_setup(1))
			{
				QuadRotor_motor1_start();
				QuadRotor_motor1_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			if(!is_setup(2))
			{
				QuadRotor_motor2_start();
				QuadRotor_motor2_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			if(!is_setup(3))
			{
				QuadRotor_motor3_start();
				QuadRotor_motor3_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			if(!is_setup(4))
			{
				QuadRotor_motor4_start();
				QuadRotor_motor4_setDuty(0.0f);
				timekeeper_delay(1000U);
			}
			QuadRotor_motor1_setDuty(pwr);
			QuadRotor_motor2_setDuty(pwr);
			QuadRotor_motor3_setDuty(pwr);
			QuadRotor_motor4_setDuty(pwr);			
		}
		else
		{
			printf("ERROR, invalid motor selection!! Valid options: 1, 2, 3, 4 or all\r\n");
		}
	}
	printf("\r\n...done\r\n");
}

void calibrate_esc(int argc, char** argv)
{
	_enable_interrupts();
	if(argc == 1)
	{
		if(strncmp(argv[0], "1", 1) == 0)
		{
			esc_cal(1);
		}
		else if(strncmp(argv[0], "2", 1) == 0)
		{
			esc_cal(2);
		}
		else if(strncmp(argv[0], "3", 1) == 0)
		{
			esc_cal(3);			
		}
		else if(strncmp(argv[0], "4", 1) == 0)
		{
			esc_cal(4);			
		}
		else
		{
			printf("ERROR: Out-of-bounds range of motor number!! Valid numbers: 1-4\r\n");
		}
	}
	else
	{
		printf("Usage: esc_cal [1/2/3/4]");
	}
}

void flight_app(int argc, char** argv)
{
	gnc_control_mode m;
	int fc_mode = 0;

	float attitude_kP = 3.2f;
	float attitude_kD = 0.00f;

	float rate_kP = 0.2f;

	if(argc >= 3)
	{
		sscanf(argv[0], "%f", &attitude_kP);
		sscanf(argv[1], "%f", &attitude_kD);
		sscanf(argv[2], "%f", &rate_kP);

		printf("Starting flight control application with Attitude P gain %f, D gain %f and Rate P gain %f\r\n", attitude_kP, attitude_kD, rate_kP);

		if(argc >= 4)
		{
			if(strncmp(argv[3], "rate", 4) == 0)
			{
				m = MODE_RATE_CONTROL;
				printf("Using angular rate control mode..\r\n");
			}
			else if(strncmp(argv[3], "attitude", 8) == 0)
			{
				m = MODE_ATTITUDE_CONTROL;
				printf("Using angular attitude control mode..\r\n");
			}
			else if(strncmp(argv[3], "att_height_heading", 18) == 0)
			{
				m = MODE_ATTITUDE_HEIGHT_HEADING_CONTROL;
				printf("Using angular attitude, height and heading control mode..\r\n");
			}
			else if(strncmp(argv[3], "att_height", 10) == 0)
			{
				m = MODE_ATTITUDE_HEIGHT_CONTROL;
				printf("Using angular attitude and closed-loop height mode..\r\n");
			}
			else if(strncmp(argv[3], "vel", 3) == 0)
			{
				m = MODE_VELOCITY_CONTROL;
				printf("Using lateral and vertical velocity control mode..\r\n");
			}
			else
			{
				printf("Invalid control mode argument!! Exiting now...\r\n");
				return;
			}
		}
	}

	update_cmds = 0U;

	// Initialize PWM channels for 4 motors and set all motors to stop by default:
	QuadRotor_PWM_init();
	QuadRotor_motor1_start();
	QuadRotor_motor2_start();
	QuadRotor_motor3_start();
	QuadRotor_motor4_start();

	/*
		All motors stopped by default:
	 */
	QuadRotor_motor1_setDuty(0.0f);
	QuadRotor_motor2_setDuty(0.0f); 
	QuadRotor_motor3_setDuty(0.0f);
	QuadRotor_motor4_setDuty(0.0f);

	rt_telemetry_init_channel(&telem0, &ftdi_dbg_port);

	float roll_cmd, pitch_cmd, yaw_cmd;

	roll_cmd = 0.0f;
	pitch_cmd = 0.0f;
	yaw_cmd = 0.0f;

	throttle_value_common = 0.0f;

	double motor_output_commands[4];

	motor_output_commands[0] = 0.0f;
	motor_output_commands[1] = 0.0f;
	motor_output_commands[2] = 0.0f;
	motor_output_commands[3] = 0.0f;

	gnc_innerloop_flag = 0U;

	float imudata_telemetry_output[7];

	board_led_on(LED1);
	board_led_off(LED2);

	gnc_init(attitude_kP, attitude_kD, rate_kP, m);

	board_led_off(LED1);
	timekeeper_delay(1000U);

	float att_data[3];
	att_data[2] = 0.0f;

	float sensor_data[6];

	float motor_vals[4];

	float mission_time_sec = 0.0f;
	int32_t mission_time_msec = 0;

	init_rc_inputs(&rc_data, 0);
	
	float roll_rate_cmd_local, pitch_rate_cmd_local, yaw_rate_cmd_local, throttle_value_common_local;
	roll_rate_cmd_local = 0.0f;
	pitch_rate_cmd_local = 0.0f;
	yaw_rate_cmd_local = 0.0f;
	throttle_value_common_local = 0.0f;

	uint8_t i = 0U;

	float height_sensor_reading = 0.0f;
	float height_closedloop_throttle_cmd = 0.0f;
	float max_height_cmd = 2.0f;
	float h_est_telem_msg[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	float heading_sensor_reading = 0.0f;
	float yaw_sensor_reading = 0.0f;

	float roll_adj = 0.0f;
	float pitch_adj = 0.0f;
	float yaw_adj = 0.0f;

	float mcmds[4];
	float height_estimate = 0.0f;
	float vert_vel_estimate = 0.0f;
	float vert_accel_estimate = 0.0f;

	while(1)
	{
		gnc_enable();
		if(update_cmds)
		{
			/* 
				To actually query the IMU for raw data and populate internal GNC data structs.
				This function call is blocking and takes about 600 us to run.
			 */
			gnc_update_vehicle_state();
			_disable_interrupts();

				update_cmds = 0;

				// gnc_attitude_rate_controller_update(throttle_value_common_local); // Open-loop height
				gnc_attitude_rate_controller_update(height_closedloop_throttle_cmd); // Closed-loop height
				gnc_get_actuator_commands(&mcmds[0]);

				QuadRotor_motor1_setDuty(mcmds[0]);
				QuadRotor_motor2_setDuty(mcmds[1]);
				QuadRotor_motor3_setDuty(mcmds[2]);
				QuadRotor_motor4_setDuty(mcmds[3]);
				
				publish_motor_commands(&mcmds[0]);
			
			_enable_interrupts();
		}
		/*
			Synchronous (i.e. cyclic) events scheduled by timer (RTI or PWM) subsystem:
		 */

		if(get_flag_state(flag_1000hz) == STATE_PENDING)
		{
			reset_flag(flag_1000hz);

			if(get_flag_state(flag_1hz) == STATE_PENDING)
			{
				reset_flag(flag_1hz);
				board_led_toggle(LED2); // Toggle green LED on interface board as heartbeat
			}

			if(get_flag_state(flag_10hz) == STATE_PENDING)
			{
				reset_flag(flag_10hz);				
				rc_input_validity_watchdog_callback(); // Check if sufficient number of RC rising edges have occurred in last 100 ms
			}

			if(get_flag_state(flag_100hz) == STATE_PENDING)
			{
				reset_flag(flag_100hz);
				gnc_attitude_controller_update(roll_cmd, pitch_cmd, yaw_cmd);
			}
			
			if(get_flag_state(flag_200hz) == STATE_PENDING)
			{
				board_led_toggle(LED1);

				// Reset flag for next cycle:				
				reset_flag(flag_200hz);

				// Send telemetry items as configured:

				#ifdef SEND_MPU_IMU_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"imu", 3, sensor_data, 6);
				#endif
				#ifdef SEND_STATE_VECTOR_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"state", 5, att_data, 2);
				#endif

				gnc_get_height_state_vector(&height_estimate, &vert_vel_estimate, &vert_accel_estimate);
				height_closedloop_throttle_cmd = gnc_get_height_controller_throttle_command(throttle_value_common_local*max_height_cmd);

				_disable_interrupts();
					roll_rate_cmd = roll_rate_cmd_local;
					pitch_rate_cmd = pitch_rate_cmd_local;
					yaw_rate_cmd = yaw_rate_cmd_local;
				_enable_interrupts();

				#ifdef SEND_MOTOR_CMDS_TELEMETRY
				motor_vals[0] = (float)motor_output_commands[0];
				motor_vals[1] = (float)motor_output_commands[1];
				motor_vals[2] = (float)motor_output_commands[2];
				motor_vals[3] = (float)motor_output_commands[3];

				send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"cmds", 4, motor_vals, 4);
				#endif

				if(get_flag_state(flag_20hz) == STATE_PENDING)
				{
					reset_flag(flag_20hz);

					#ifdef SEND_HEIGHT_SENSOR_TELEMETRY
						send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"sf11c", 5, &height_sensor_reading, 1);
					#endif
					
					yaw_sensor_reading = get_last_can_heading_msg();
				}
				
				if(get_flag_state(flag_50hz) == STATE_PENDING)
				{
				#ifdef SEND_CAN_ROLL_PITCH
					// publish_roll_pitch(radians_to_degrees(sd.state_vector.roll), radians_to_degrees(sd.state_vector.pitch));
				#endif
					reset_flag(flag_50hz);

					#ifdef SEND_CAN_YAW_HEIGHT
						publish_yaw_height_estimate(yaw_sensor_reading, height_estimate);
					#endif

					#ifdef SEND_CAN_VERT_VEL
						publish_vert_velocity_estimate(vert_vel_estimate);
					#endif

					#ifdef SEND_HEIGHT_ESTIMATOR_TELEMETRY
						h_est_telem_msg[0] = height_estimator.height_estimated;
						h_est_telem_msg[1] = height_estimator.vertical_velocity_estimated;
						h_est_telem_msg[2] = height_closedloop_throttle_cmd;
						h_est_telem_msg[3] = throttle_value_common_local*max_height_cmd;
						h_est_telem_msg[4] = height_sensor_reading;
						h_est_telem_msg[5] =  gnc_get_vertical_dynamic_acceleration();
						h_est_telem_msg[6] =  sd.roll;
						h_est_telem_msg[7] =  sd.pitch;
						send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"h_est", 5, h_est_telem_msg, 8);
					#endif

					#ifdef SEND_HEADING_TELEMETRY
						send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"heading", 7, &heading_sensor_reading, 1);
					#endif

					#ifdef SEND_BNO_SERIAL_STATS_TELEMETRY
						send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"bno_tot", 7, &bno_bytes_total, 1);
					#endif
						// Get RC Joystick data from last reception:

						get_rc_input_values(&rc_data);

						if(rc_data.mode_switch_channel_validity == CHANNEL_VALID)
						{
							#ifdef SEND_RC_INPUT_TELEMETRY
								if(get_ch5_mode(rc_data) == MODE_NORMAL)
								{
									send_telem_msg_string_blocking(&telem0, (uint8_t *)"CH5", 3, (uint8_t *)"Normal", 6);
								}
								if(get_ch5_mode(rc_data) == MODE_FAILSAFE)
								{
									send_telem_msg_string_blocking(&telem0, (uint8_t *)"CH5", 3, (uint8_t *)"FAILSAFE", 8);
								}
							#endif
								roll_cmd = rc_data.roll_channel_value;
								pitch_cmd = rc_data.pitch_channel_value;
								yaw_cmd = rc_data.yaw_channel_value * -1.0f;
								throttle_value_common_local = 0.50f*(rc_data.vertical_channel_value + 1.0f);
						}
						else
						{
							QuadRotor_motor1_setDuty(0.0f);
							QuadRotor_motor2_setDuty(0.0f);
							QuadRotor_motor3_setDuty(0.0f);
							QuadRotor_motor4_setDuty(0.0f);
							// Wait for serial transmit of telemetry to complete, and give ESCs a chance to latch the last valid command prior to PWM shutdown.
							QuadRotor_motor1_stop();
							QuadRotor_motor2_stop();
							QuadRotor_motor3_stop();
							QuadRotor_motor4_stop();
							board_led_on(LED1); 	// Red ERROR LED = ON
							board_led_off(LED2); 	// Green OK LED = OFF
							printf("Exiting due to LOS...\r\n");
							return;
						}
				}
				/*
					Emergency-stop all motors upon loss of signal. In the future this may become something a bit more graceful...
				 */
				#ifdef SEND_RC_INPUT_TELEMETRY
					send_telem_msg_string_blocking(&telem0, (uint8_t *)"rcvalid", 7, "FALSE\r\n", 7);
				#endif
			}
		}
	}
/* USER CODE END */
}

int main(void)
{
	/*Globally disable R4 IRQs AND FIQs:*/
	_disable_interrupts();

	cpu_init();

	// Initialize the MibSPI in GPIO mode to be able to use system LEDs on interface board:
	board_led_init();

	// Set up timing flags for timed tasks in various apps:
	init_mission_timekeeper();

	flag_200hz = create_flag(4U);
	flag_20hz = create_flag(49U);
	flag_50hz = create_flag(19U);
	flag_1000hz = create_flag(0U);
	flag_100hz = create_flag(9U);
	flag_10hz = create_flag(99U);
	flag_1hz = create_flag(999U);

	// Initialize the shell and underlying serial port API at 460800 baud, 8n1 on SCI1:
	setup_system_shell();

	printf("Initialized all drivers...\r\n");

	printf("Initialized system shell...\r\n now installing apps...\r\n");

	if(install_cmd("led_ctl", ledcontrol) < 0)
	{
		printf("Install error in command led_ctl\r\n");
	}

	if(install_cmd("clear", shell_clear) < 0)
	{
		printf("Install error in command clear\r\n");
	}

	if(install_cmd("motorset", motorcontrol) < 0)
	{
		printf("Install error in command motorset\r\n");
	}

	if(install_cmd("fc_app", flight_app) < 0)
	{
		printf("Install error in command fc_app\r\n");
	}

	if(install_cmd("esc_cal", calibrate_esc) < 0)
	{
		printf("Install error in command esc_cal\r\n");
	}

	printf("...Done, enabling interrupts.\r\n");

	_enable_interrupts();

	/*
		Autostarted apps go in sequence below. Once these exit the system will simply go back to running the shell 
		and processing further user input:
	 */

	flight_app(0, NULL);

	while(1)
	{
		shell_run(); // Keep prompting for more commands and processing appropriately...
	}

	// Never get here, or Bad Things (tm) happen :)
}
