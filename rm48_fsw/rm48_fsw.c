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
// #include "can_comms.h"
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

static void check_saturation(float *num, float lower, float upper)
{
	if(*num > upper)
	{
		*num = upper;
	}
	if(*num < lower)
	{
		*num = lower;
	}
}

void flight_app(int argc, char** argv)
{
	m1_cmd = 0.0f;
	m2_cmd = 0.0f;
	m3_cmd = 0.0f;
	m4_cmd = 0.0f;

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
	observation rd;
	complementary_filter_struct sd;

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

	height_kalman_data_struct height_estimator;
	gnc_height_kalman_struct_init(&height_estimator, 0.005f, 0.05f);
	float height_sensor_reading = 0.0f;
	float height_closedloop_throttle_cmd = 0.0f;
	float max_height_cmd = 2.0f;
	float h_est_telem_msg[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	float heading_sensor_reading = 0.0f;
	float yaw_sensor_reading = 0.0f;

	float roll_adj = 0.0f;
	float pitch_adj = 0.0f;
	float yaw_adj = 0.0f;

	float bno_attitude[3];

	while(1)
	{
		gnc_enable();
		if(update_cmds)
		{
			/* 
				To actually query the IMU for raw data and populate internal GNC data structs.
				This function call is blocking and takes about 600 us to run.
			 */
			gnc_get_vehicle_state();
			_disable_interrupts();

				update_cmds = 0;
				roll_adj = 0.3f*(roll_rate_cmd-degrees_to_radians(sd.imu_data.gyro_data[0]));
				pitch_adj = 0.3f*(pitch_rate_cmd-degrees_to_radians(sd.imu_data.gyro_data[1]));
				yaw_adj = 0.2f*(yaw_rate_cmd-degrees_to_radians(sd.imu_data.gyro_data[2]));

				m1_cmd = throttle_value_common_local + roll_adj*0.5f + pitch_adj*0.5f + yaw_adj;
				m2_cmd = throttle_value_common_local - roll_adj*0.5f + pitch_adj*0.5f - yaw_adj;
				m3_cmd = throttle_value_common_local - roll_adj*0.5f - pitch_adj*0.5f + yaw_adj;
				m4_cmd = throttle_value_common_local + roll_adj*0.5f - pitch_adj*0.5f - yaw_adj;

				check_saturation(&m1_cmd, 0.01f, 0.98f);
				check_saturation(&m2_cmd, 0.01f, 0.98f);
				check_saturation(&m3_cmd, 0.01f, 0.98f);
				check_saturation(&m4_cmd, 0.01f, 0.98f);
			
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
				// printf("%f %f\r\n", roll_cmd, pitch_cmd);
				// printf("Roll: %f, Pitch: %f\r\n", sd.state_vector.roll, sd.state_vector.pitch);
			}

			if(get_flag_state(flag_100hz) == STATE_PENDING)
			{
				reset_flag(flag_100hz);
				get_last_attitude(&bno_attitude[0]);
			}
			
			
			// For telemetry purposes:
			gnc_get_raw_sensor_data(&rd);
			gnc_get_state_vector_data(&sd);
			att_data[0] = sd.state_vector.roll;
			att_data[1] = sd.state_vector.pitch;

			sensor_data[0] = 0.0f;//rd.x_accel;
			sensor_data[1] = 0.0f;//rd.y_accel;
			sensor_data[2] = 0.0f;//rd.z_accel;

			sensor_data[3] = 0.0f;//rd.roll_gyro;
			sensor_data[4] = 0.0f;//rd.pitch_gyro;
			sensor_data[5] = 0.0f;//rd.yaw_gyro;
			
			if(get_flag_state(flag_200hz) == STATE_PENDING)
			{
				board_led_toggle(LED1);

				// Reset flag for next cycle:				
				reset_flag(flag_200hz);

				// Send telemetry items as configured:
				#ifdef SEND_CAN_ROLL_PITCH
					publish_roll_pitch(bno_attitude[0],bno_attitude[1]);//radians_to_degrees(sd.state_vector.roll), radians_to_degrees(sd.state_vector.pitch));
				#endif

				#ifdef SEND_MPU_IMU_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"imu", 3, sensor_data, 6);
				#endif
				#ifdef SEND_STATE_VECTOR_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"state", 5, att_data, 2);
				#endif

				// Run height estimation Kalman filter:
				gnc_height_kalman_update(&height_estimator, height_sensor_reading, gnc_get_vertical_dynamic_acceleration(), sd.state_vector.roll, sd.state_vector.pitch);

				// Obtain new height throttle command:
				height_closedloop_throttle_cmd = gnc_get_height_controller_throttle_command(throttle_value_common_local*max_height_cmd, 
																						height_estimator.height_estimated,
																						height_estimator.vertical_velocity_estimated);

				// Run GNC angular outer loop control to generate rate commands for inner loop:
				// gnc_vehicle_stabilization_outerloop_update(roll_cmd, pitch_cmd, yaw_cmd,
				// 											&roll_rate_cmd_local, &pitch_rate_cmd_local, &yaw_rate_cmd_local);

				// Simple outerloop update :)
				// error = command-measurement
				roll_rate_cmd_local = 1.8f*((roll_cmd*0.3f)-degrees_to_radians(bno_attitude[0]));// sd.state_vector.roll);
				pitch_rate_cmd_local = 1.8f*((pitch_cmd*-0.3f)-degrees_to_radians(bno_attitude[1]));//sd.state_vector.pitch); // Since pitch stick is opposite pitch angle convention for aircraft body frame
				yaw_rate_cmd_local = 0.3f*(yaw_cmd*-4.5f);//-degrees_to_radians(sd.imu_data.gyro_data[2]));


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
					
					height_sensor_reading = get_last_can_height_msg();
					yaw_sensor_reading = get_last_can_heading_msg();
				}
				
				if(get_flag_state(flag_50hz) == STATE_PENDING)
				{
					reset_flag(flag_50hz);

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

void test_comp_filter(int argc, char** argv)
{
	printf("\r\nThis app will run the second-order complementary filter until the RC controller is switched off.\r\n");
	printf("Press p to proceed or e to exit\r\n");
	char c = ' ';
	while(c != 'p' && c != 'e')
	{
		scanf("%c", &c);
		if(c == 'e')
		{
			return;
		}
		if(c == 'p')
		{
			printf("Proceeding...\r\n");
			break;
		}
	}
  	_disable_interrupts();

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

	_enable_interrupts();

	timekeeper_delay(3000U);

	complementary_filter_struct s;
	init_complementary_filter(&s, SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS,
	                          0.010f, 0.8f, 1.0f, MODE_2NDORDER_COMPFILTER);

	init_rc_inputs(&rc_data, 0);

	timekeeper_delay(1000U);

	printf("Setup complete, starting main loop...\r\n");

	while(1)
	{
		// Get RC Joystick data from last reception:

		get_rc_input_values(&rc_data);

		if(rc_data.mode_switch_channel_validity == CHANNEL_VALID)
		{
			QuadRotor_motor1_setDuty(0.50f*(rc_data.vertical_channel_value + 1.0f));
			QuadRotor_motor2_setDuty(0.50f*(rc_data.vertical_channel_value + 1.0f));
			QuadRotor_motor3_setDuty(0.50f*(rc_data.vertical_channel_value + 1.0f));
			QuadRotor_motor4_setDuty(0.50f*(rc_data.vertical_channel_value + 1.0f));

			if(get_flag_state(flag_100hz) == STATE_PENDING)
			{
				reset_flag(flag_100hz);
				update_complementary_filter(&s);
				#ifdef SEND_CAN_ROLL_PITCH
					publish_roll_pitch(radians_to_degrees(s.state_vector.roll), radians_to_degrees(s.state_vector.pitch));
				#endif
			}

			if(get_flag_state(flag_10hz) == STATE_PENDING)
			{
				reset_flag(flag_10hz);
				rc_input_validity_watchdog_callback();
				printf("Roll: %f Pitch: %f\r\n", radians_to_degrees(s.state_vector.roll), radians_to_degrees(s.state_vector.pitch));
				// printf("Roll: %f Pitch: %f\r\n", radians_to_degrees(s.state_vector.roll_rate), radians_to_degrees(s.state_vector.pitch_rate));
			}
		}
		else
		{
			printf("\r\nUSER button pressed, exiting...\r\n");
			return;
		}
	}
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

	if(install_cmd("compfilter", test_comp_filter) < 0)
	{
		printf("Install error in command compfilter\r\n");
	}

	printf("...Done, enabling interrupts.\r\n");

	_enable_interrupts();

	flight_app(0, NULL);

	while(1)
	{
		shell_run();
	}
}
