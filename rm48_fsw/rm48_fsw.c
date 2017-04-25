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
#include "sf10_reader.h"
#include "bno055_reader.h"

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

/*
	Central define that enables or disables the operation of motors under any circumstance.
	Uncomment this if you want the motors to be able to rotate:
 */
// #define ENABLE_MOTORS		1
/*
	Disable main program loop and run the ESC PWM channels through a pre-determined sequence to
	calibrate them:
 */
// #define ESC_CAL_MODE	1
/* 
	Disable main program loop and run an infinite loop emitting telemetry packets
	at about 50 Hz for test purposes:
 */
// #define TEST_TELEM			1

/*
	Uncomment the following to enable the RC calibration and arming sequence, and 
	subsequent control of vehicle using the RC controller:
 */
// #define ENABLE_RC_CONTROL 	1

/* USER CODE END */

/*
	Some unavoidable global definitions that are also accessed by functions called from interrupt
	handlers in interrupts.c:
 */

serialport ftdi_dbg_port, tm4c_comms_aux_port;
serialport *ftdi_dbg_port_ptr;
serialport *tm4c_comms_aux_port_ptr;

serialport tm4c_port1, tm4c_port2, tm4c_port3;
serialport *tm4c_port1_ptr;
serialport *tm4c_port2_ptr;
serialport *tm4c_port3_ptr;

rt_telemetry_comm_channel telem0;

float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
float throttle_value_common;

uint8_t gnc_innerloop_flag;

int main(void)
{
/* USER CODE BEGIN (3) */

	/*Globally disable R4 IRQs AND FIQs:*/
	_disable_interrupts();

	cpu_init();

	while(1);

	board_led_init(); // Initialize the MibSPI in GPIO mode to be able to use system LEDs on interface board.

	// Initialize all configured serial ports, including HAL subsystems + data structs for asynch tx/rx:

	ftdi_dbg_port_ptr = &ftdi_dbg_port;
	tm4c_comms_aux_port_ptr = &tm4c_comms_aux_port;

	tm4c_port1_ptr = &tm4c_port1;
	tm4c_port2_ptr = &tm4c_port2;
	tm4c_port3_ptr = &tm4c_port3;

	serialport_init(ftdi_dbg_port_ptr, PORT1);

	// Uses TM4C UART port (virtual UART subsystem) #1:
	sf10_sensor_data_handler sf10_handler;
	sf10_hal_setup_ap2v4_serial();
	init_new_sf10_data_handler(&sf10_handler, 200U, MAX_HEIGHT_SF11_C, send_byte_to_sf10A);

	serialport_init(tm4c_port2_ptr, TM4C_PORT2);
	serialport_init(tm4c_port3_ptr, TM4C_PORT3);

	rt_telemetry_init_channel(&telem0, ftdi_dbg_port_ptr);

	#ifdef ESC_CAL_MODE
		init_mission_timekeeper();
		_enable_interrupts();
		board_led_on(LED1);
		board_led_off(LED2);
		esc_cal();
	#endif

	/*
		If ESC Calibration mode above is enabled, the program will never get here, so all subsequent procedures are irrelevant.
	 */

	/*
	* Initialize the serial port (SCI module) and enable SCI Receive interrupts:
	* (Explanation: mibSPI peripheral must be initialized as well since SCI requires mibSPI pins)
	*/	
	
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

	/*
		Initialize the PWM input functionality. Technically this just makes a duplicate call to hetInit and zeroes the
		edge counter that's used for LOS (Loss of Signal) detection functionality.
	 */
	pwm_input_init();

	init_mission_timekeeper();

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

	_enable_interrupts();

	float imudata_telemetry_output[7];

	uint8_t telemetry_gnc_200hz_flag = create_flag(4U);
	uint8_t sf10_20hz_trigger_flag = create_flag(49U);
	uint8_t rc_update_50hz_flag = create_flag(19U);
	uint8_t imu_sample_1000hz_flag = create_flag(0U);
	uint8_t rc_watchdog_10hz_flag = create_flag(99U);
	uint8_t heartbeat_1hz_flag = create_flag(999U);

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

	rc_joystick_data_struct rc_data;
	#ifdef ENABLE_RC_CONTROL
		init_rc_inputs(&rc_data);
	#endif

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

	float h_est_telem_msg[8] = {0.0f};
	// h_est_telem_msg[0] = 0.0f;
	// h_est_telem_msg[1] = 0.0f;
	// h_est_telem_msg[2] = 0.0f;

	float heading_sensor_reading = 0.0f;

	uint8_t bno_bytes_read = 0U;
	int bno_bytes_total = 0U;

	// BNO055_init();

	serialport_send_data_buffer(tm4c_port3_ptr, (uint8_t *)"Hello UART7!!\r\n", 15U);

	while(1)
	{
		#ifdef ENABLE_MOTORS
			gnc_enable();
		#else
			gnc_disable();
		#endif
		/*
			Asynchronous events (primarily serial communications using encapsulation driver subsystem):
		 */

		/*
			Process SF11/C height sensor data:
		 */
		// bytes_read = serialport_receive_data_buffer(tm4c_port2_ptr, msg_buf, 10);

		// for(i=0; i<bytes_read; ++i)
		// {
		// 	sf10_reader_callback(&sf10_handler, msg_buf[i]);				
		// }

		// if(sf10_received_new_data(&sf10_handler))
		// {
		// 	// float height_sensor_reading_raw = get_last_sf10_sensor_height(&sf10_handler);
		// 	// height_sensor_reading = sf10_reader_check_measurement(height_sensor_reading_raw);

		// 	height_sensor_reading = get_last_sf10_sensor_height(&sf10_handler);
		// }

		/*
			Process BNO055 Heading sensor data:
		 */

		// bno_bytes_read = serialport_receive_data_buffer(tm4c_port2_ptr, msg_buf, 30U);

		// for(i=0; i<bno_bytes_read; ++i)
		// {
		// 	BNO055_interrupt_handler(msg_buf[i]);
		// }

		// if(BNO055_received_new_data())
		// {
		// 	imu_data bno055_reading;
		// 	BNO055_get_imu_data(&bno055_reading);
		// 	heading_sensor_reading = bno055_reading.heading;
		// 	bno_bytes_total += 1;
		// }

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/*
			Synchronous (i.e. cyclic) events scheduled by timer (RTI or PWM) subsystem:
		 */

		if(get_flag_state(heartbeat_1hz_flag) == STATE_PENDING)
		{
			reset_flag(heartbeat_1hz_flag);
			board_led_toggle(LED2); // Toggle green LED on interface board as heartbeat
		}

		if(get_flag_state(imu_sample_1000hz_flag) == STATE_PENDING)
		{
			// board_led_toggle(LED1);
			reset_flag(imu_sample_1000hz_flag);

			if(get_flag_state(rc_watchdog_10hz_flag) == STATE_PENDING)
			{
				reset_flag(rc_watchdog_10hz_flag);				
				rc_input_validity_watchdog_callback(); // Check if sufficient number of RC rising edges have occurred in last 100 ms
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
			
			if(get_flag_state(telemetry_gnc_200hz_flag) == STATE_PENDING)
			{
				// Reset flag for next cycle:				
				reset_flag(telemetry_gnc_200hz_flag);

				// board_led_toggle(LED1);

				// Send telemetry items as configured:
				#ifdef SEND_MPU_IMU_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"imu", 3, sensor_data, 6);
				#endif
				#ifdef SEND_STATE_VECTOR_TELEMETRY
					send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"state", 5, att_data, 2);
				#endif

				// Run height estimation Kalman filter:
				gnc_height_kalman_update(&height_estimator, height_sensor_reading, gnc_get_vertical_dynamic_acceleration(), sd.roll, sd.pitch);

				// send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"h_est", 5, &(height_estimator.height_estimated), 1);

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

				if(get_flag_state(sf10_20hz_trigger_flag) == STATE_PENDING)
				{
					reset_flag(sf10_20hz_trigger_flag);

					#ifdef SEND_HEIGHT_SENSOR_TELEMETRY
						send_telem_msg_n_floats_blocking(&telem0, (uint8_t *)"sf11c", 5, &height_sensor_reading, 1);
					#endif
					
					board_led_toggle(LED1);
					// request_sf10_sensor_update(&sf10_handler);
				}
				
				if(get_flag_state(rc_update_50hz_flag) == STATE_PENDING)
				{
					reset_flag(rc_update_50hz_flag);

					// BNO055_trigger_get_data();

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

					#ifdef ENABLE_RC_CONTROL
						// Get RC Joystick data from last reception:

						get_rc_input_values(&rc_data);

						if(get_ch5_mode(rc_data)==MODE_FAILSAFE)
						{
							gnc_integral_enable();
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
							/*
								Emergency-stop all motors upon loss of signal. In the future this may become something a bit more graceful...
							 */
							#ifdef SEND_RC_INPUT_TELEMETRY
								send_telem_msg_string_blocking(&telem0, (uint8_t *)"rcvalid", 7, "FALSE\r\n", 7);
							#endif
							
							insert_delay(100);
							_disable_interrupts();
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
							while(1); 				// Block here indefinitely pending system power-off/reset by user
						}
					#else
						roll_cmd = 0.0f;
						pitch_cmd = 0.0f;
						yaw_cmd = 0.0f;
						throttle_value_common_local = 0.0f;
					#endif
				}
			}
		}
	}
/* USER CODE END */
}

/* USER CODE BEGIN (4) */
/* USER CODE END */
	
