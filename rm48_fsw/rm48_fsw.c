/*Standard HAL (Hardware Abstraction Layer) includes:*/
#include "hal_common_includes.h"
#include "interrupt_utils.h"

// User/flight software libraries:
#include "basic_pid_controller.h"
#include "iir_filters.h"
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
#include "can_comms.h"
#include "cpu_hal_interface.h"
#include "system_shell.h"	
#include "can_comms.h"

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

	(c) 2016, Abhimanyu Ghosh
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
volatile uint8_t flag_200hz;
volatile uint8_t flag_1000hz;

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

	// serialport_hal_init();
	// serialport_init(&ftdi_dbg_port, PORT1);
	rt_telemetry_init_channel(&telem0, &ftdi_dbg_port);

	_enable_interrupts();

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

	gnc_init();
	gnc_raw_data rd;
	gnc_state_data sd;

	board_led_off(LED1);
	timekeeper_delay(1000U);

	float att_data[3];
	att_data[2] = 0.0f;

	float sensor_data[6];

	float motor_vals[4];

	float mission_time_sec = 0.0f;
	int32_t mission_time_msec = 0;

	#ifdef TEST_TELEM
		float flt_test[2];
		flt_test[0] = 0.0f;
		flt_test[1] = 0.0f;

		int32_t int_test[2];
		int_test[0] = 0;
		int_test[1] = 0;

		uint8_t *str_test[2] = {"hello0", "hello1"};

		while(1)
		{
			mission_time_sec = get_mission_time_sec();
			mission_time_msec = get_mission_time_msec();
			flt_test[0] = mission_time_sec;
			int_test[0] = mission_time_msec;

			if(flt_test[1] < 1.0f)
			{
				flt_test[1] += 0.01f;
			}
			else
			{
				flt_test[1] = 0.0f;
			}

			if(int_test[1] < 100)
			{
				int_test[1] += 1;
			}
			else
			{
				int_test[1] = 0;
			}

			send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"flt", 3, flt_test, 2);
			send_telem_msg_n_ints_blocking(&telem0, (uint8_t *)"int", 3, int_test, 2);
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"str0", 4, str_test[0], 6);
			send_telem_msg_string_blocking(&telem0, (uint8_t *)"str1", 4, str_test[1], 6);

			timekeeper_delay(5U);
		}
	#endif

	init_rc_inputs(&rc_data, 0);
	
	float roll_rate_cmd_local, pitch_rate_cmd_local, yaw_rate_cmd_local, throttle_value_common_local;
	roll_rate_cmd_local = 0.0f;
	pitch_rate_cmd_local = 0.0f;
	yaw_rate_cmd_local = 0.0f;
	throttle_value_common_local = 0.0f;

	uint8_t msg_buf[30];
	uint8_t i = 0U;
	uint32_t bytes_read = 0U;

	float height_sensor_reading = 0.0f;

	height_kalman_data_struct height_estimator;
	gnc_height_kalman_struct_init(&height_estimator, 0.005f, 0.05f);
	float height_closedloop_throttle_cmd = 0.0f;
	float max_height_cmd = 2.0f;

	float h_est_telem_msg[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	float heading_sensor_reading = 0.0f;

	uint8_t bno_bytes_read = 0U;
	int bno_bytes_total = 0U;
	float yaw_sensor_reading = 0.0f;

	while(1)
	{
		gnc_enable();
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
				_disable_interrupts();
			}
			
			/* 
				To actually query the IMU for raw data and populate internal GNC data structs.
				This function call is blocking and takes about 600 us to run.
			 */
			gnc_get_vehicle_state();
			
			// For telemetry purposes:
			gnc_get_raw_sensor_data(&rd);
			gnc_get_state_vector_data(&sd);
			att_data[0] = sd.roll;
			att_data[1] = sd.pitch;

			sensor_data[0] = rd.x_accel;
			sensor_data[1] = rd.y_accel;
			sensor_data[2] = rd.z_accel;

			sensor_data[3] = rd.roll_gyro;
			sensor_data[4] = rd.pitch_gyro;
			sensor_data[5] = rd.yaw_gyro;
			
			if(get_flag_state(flag_200hz) == STATE_PENDING)
			{
				// Reset flag for next cycle:				
				reset_flag(flag_200hz);

				board_led_toggle(LED1);

				// Send telemetry items as configured:
				#ifdef SEND_MPU_IMU_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"imu", 3, sensor_data, 6);
				#endif
				#ifdef SEND_STATE_VECTOR_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"state", 5, att_data, 2);
				#endif

				// Run height estimation Kalman filter:
				gnc_height_kalman_update(&height_estimator, height_sensor_reading, gnc_get_vertical_dynamic_acceleration(), sd.roll, sd.pitch);

				// // Obtain new height throttle command:
				height_closedloop_throttle_cmd = gnc_get_height_controller_throttle_command(throttle_value_common_local*max_height_cmd, 
																						height_estimator.height_estimated,
																						height_estimator.vertical_velocity_estimated);

				// Run GNC angular outer loop control to generate rate commands for inner loop:
				gnc_vehicle_stabilization_outerloop_update(roll_cmd, pitch_cmd, yaw_cmd,
															&roll_rate_cmd_local, &pitch_rate_cmd_local, &yaw_rate_cmd_local);

				_disable_interrupts();
					roll_rate_cmd = roll_rate_cmd_local;
					pitch_rate_cmd = pitch_rate_cmd_local;
					yaw_rate_cmd = yaw_rate_cmd_local;
					throttle_value_common = throttle_value_common_local; // For open-loop height control!!
					// throttle_value_common = height_closedloop_throttle_cmd; // For closed-loop height control!!
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
					
					board_led_toggle(LED1);
					height_sensor_reading = get_last_can_height_msg();
					yaw_sensor_reading = get_last_can_heading_msg();
				}
				
				if(get_flag_state(flag_50hz) == STATE_PENDING)
				{
					reset_flag(flag_50hz);

					#ifdef SEND_CAN_ROLL_PITCH
						publish_roll_pitch(sd.roll, sd.pitch);
					#endif

					#ifdef SEND_CAN_YAW_HEIGHT
						publish_yaw_height_estimate(yaw_sensor_reading, height_estimator.height_estimated);
					#endif

					#ifdef SEND_CAN_VERT_VEL
						publish_vert_velocity_estimate(height_estimator.vertical_velocity_estimated);
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

						if(get_ch5_mode(rc_data)==MODE_FAILSAFE)
						{
							gnc_integral_disable();
							set_controller_mode(MODE_ANGULAR_POSITION_CONTROL);
						}
						else
						{
							gnc_integral_enable();
							set_controller_mode(MODE_ANGULAR_RATE_CONTROL);
						}
						if(rc_data.mode_switch_channel_validity == CHANNEL_VALID)
						{
							#ifdef SEND_RC_INPUT_TELEMETRY
								// send_telem_msg_string_blocking(&telem0, (uint8_t *)"rcvalid", 7, "TRUE\r\n", 6);
								// send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"roll", 4, &(rc_data.roll_channel_value), 1);
								// send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"pitch", 5, &(rc_data.pitch_channel_value), 1);
								// send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"yaw", 3, &(rc_data.yaw_channel_value), 1);
								// send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"height", 6, &(rc_data.vertical_channel_value), 1);

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


void main(void)
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

	while(1)
	{
		shell_run();
	}
}
