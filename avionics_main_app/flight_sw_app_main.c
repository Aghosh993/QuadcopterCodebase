/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
	C Standard Library/Newlib includes:
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
	HAL/HW-specific includes:
 */

#include <hal_common_includes.h>

/*
	Avionics software-specific includes:
 */

#include <interrupts.h>
#include <mission_timekeeper.h>
#include <imu.h>
#include <pwm_input.h>
#include <QuadRotor_PWM.h>
#include <comp_filter.h>
#include <pid_controller.h>
#include <lidar_lite_v1.h>
#include <vehicle_state_machine.h>

// #define DISABLE_ALL_MOTORS	1

/*
	Shamelessly stolen from I2C example in libopencm3-examples,
	but sharing is caring, right? Right? Okay.
 */

#define LBLUE2 GPIOE, GPIO12

/*
	Approx active LOC (Lines of code: ~3200)
 */

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA2,3 				-> 			USART2
	PE8,9,10,11,12,13,15 -> 	LEDs
	PB6,7 			-> 			I2C1
	PA5,6,7 		-> 			SPI1 MISO,MOSI,CLK
	PE3 			-> 			SPI1 CS (user-controlled) for L3GD20
	PD3 			-> 			Timer2 Input capture
	PC6 			-> 			Timer3 Input capture
	PD12 			-> 			Timer4 Input capture
	PA15 			->			Timer8 Input capture
	PA8,9,10,PE14 	-> 			Timer1 PWM Output
 */

/*
 A basic routine to adjust system clock settings to get SYSCLK
 to 64 MHz, and have AHB buses at 64 MHz, and APB Bus at 32 MHz (its max speed)
 Copied from:
 https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f3/stm32f3-discovery/adc/adc.c
 */

/*
	External oscillator required to clock PLL at 72 MHz:
 */
static void set_system_clock(void)
{
	rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
}

static void led_gpio_setup(void)
{
	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Set GPIO8 and 12 (in GPIO port E) to 'output push-pull'. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO12);
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

#ifdef INCLUDE_PWM_TEST_SHELL

static void pwm_test_shell(void)
{
	float pwm_setval = 1.0f;
	char cmd;
	uint8_t err = 0U;
	while(1)
	{
		scanf("%c", &cmd);
		switch(cmd)
		{
			case 'p':
			err=0U;
			pwm_setval += 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'n':
			err=0U;
			pwm_setval -= 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'o':
			err=0U;
			pwm_setval += 0.10f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'b':
			err=0U;
			pwm_setval -= 0.10f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'a':
			err=0U;
			QuadRotor_motor1_start();
			QuadRotor_motor2_start();
			QuadRotor_motor3_start();
			QuadRotor_motor4_start();
			break;
			case 's':
			err=0U;
			QuadRotor_motor1_stop();
			QuadRotor_motor2_stop();
			QuadRotor_motor3_stop();
			QuadRotor_motor4_stop();
			break;
			case 'z':
			err=0U;
			pwm_setval = 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			default:
			err=1;
			break;
		}
		if(!err)
		{
			printf("%c %f\r\n", cmd, pwm_setval);
		}
		else
		{
			printf("Usage: [n,p]: Decrease/Increase PWM fine precision\r\n");
			printf("[b,o]: Decrease/Increase PWM course precision\r\n");
			printf("[a] to start output, [s] to [s]top output\r\n");
			printf("[z] to set approx 1.07 ms pulse output and keep output enabled\r\n");
		}
	}
}

#endif

/*
	A simple 3-dimensional Kalman filter (attempt) for height/other vertical axis variables
	required in height/altitude control:
 */

typedef struct {
	/*
		State variables:
	 */
	float height;
	float vertical_velocity;
	float vertical_dynamic_acceleration;

	/*
		Prediction state variables:
	 */
	float predicted_height;
	float predicted_vertical_velocity;
	float predicted_vertical_dynamic_acceleration;

	/*
		Kalman gains:
	 */
	float kalman_gain_height;
	float kalman_gain_vertical_velocity;
	float kalman_gain_vertical_dynamic_acceleration;

	/*
		Other parameters:
	 */
	float dt; // Value in seconds
	float velocity_prediction_accelerometer_weight; // Value from 0.0f to 1.0f
} vertical_state_kalman_struct;

void vertical_state_kalman_init(vertical_state_kalman_struct* str,
								float height_kalman_gain,
								float vertical_velocity_kalman_gain,
								float vertical_dynamic_acceleration_kalman_gain,
								float dt_seconds,
								float velocity_predictor_accelerometer_weight_factor);

void vertical_state_kalman_update(vertical_state_kalman_struct* vertical_state_vector, 
									filtered_quadrotor_state st);

void vertical_state_kalman_init(vertical_state_kalman_struct* str,
								float height_kalman_gain,
								float vertical_velocity_kalman_gain,
								float vertical_dynamic_acceleration_kalman_gain,
								float dt_seconds,
								float velocity_predictor_accelerometer_weight_factor)
{
	/*
		State variables:
	 */
	str->height = 0.0f;
	str->vertical_velocity = 0.0f;
	str->vertical_dynamic_acceleration = 0.0f;

	/*
		Prediction state variables:
	 */
	str->predicted_height = 0.0f;
	str->predicted_vertical_velocity = 0.0f;
	str->predicted_vertical_dynamic_acceleration = 0.0f;

	/*
		Kalman gains:
	 */
	str->kalman_gain_height = height_kalman_gain;
	str->kalman_gain_vertical_velocity = vertical_velocity_kalman_gain;
	str->kalman_gain_vertical_dynamic_acceleration = vertical_dynamic_acceleration_kalman_gain;

	/*
		Other parameters:
	 */
	str->dt = dt_seconds;
	str->velocity_prediction_accelerometer_weight = velocity_predictor_accelerometer_weight_factor;
}

/*
	A brief and quite simple concept review: https://www.ocf.berkeley.edu/~tmtong/kalman.php
 */

void vertical_state_kalman_update(vertical_state_kalman_struct* vertical_state_vector, 
									filtered_quadrotor_state st)
{
	/*
		Update state vector:
	 */
	vertical_state_vector->height = vertical_state_vector->predicted_height + 
									(vertical_state_vector->kalman_gain_height *
										(st.height-vertical_state_vector->height));

	vertical_state_vector->vertical_velocity = vertical_state_vector->predicted_vertical_velocity +
												(vertical_state_vector->kalman_gain_vertical_velocity *
													(st.vertical_velocity - vertical_state_vector->vertical_velocity));
	
	vertical_state_vector->vertical_dynamic_acceleration = vertical_state_vector->predicted_vertical_dynamic_acceleration +
															(vertical_state_vector->kalman_gain_vertical_dynamic_acceleration *
																(st.vertical_dynamic_acceleration_post_lpf - vertical_state_vector->vertical_dynamic_acceleration));

	/*
		Now perform prediction step (order of operations below are quite important, if one looks closely..:)
	 */
	vertical_state_vector->predicted_vertical_velocity = vertical_state_vector->vertical_velocity +
															(st.vertical_dynamic_acceleration_post_lpf*vertical_state_vector->dt);

	vertical_state_vector->predicted_height = vertical_state_vector->height + 
													(st.vertical_velocity * vertical_state_vector->dt);

	vertical_state_vector->predicted_vertical_dynamic_acceleration = st.vertical_dynamic_acceleration_post_lpf;
}

static volatile uint8_t angular_control_update_flag;
static volatile uint8_t height_control_update_flag;

#ifdef NESTED_HEIGHT_CONTROLLER
	static volatile uint8_t vertical_acceleration_control_update_flag;
#endif

/*
	Vehicle guidance, navigation and control (GNC) update function.
	Accepts current vehicle state pointer, inertial measurement unit (IMU) data pointer,
	vehicle state vector pointer, and pointer to user control inputs from RC joystick driver.

	Vehicle state --> Holds status of vehicle i.e. which "mode" of operation is presently being followed
	Inertial measurement unit data -> Holds data from IMU devices, scaled and offset to appropriate
										engineering units (SI.)
	Vehicle state vector --> Holds physical parameters corresponding to state of vehicle (i.e. attitude angles,
								attitude rates, lateral velocity, height, vertical dynamic acceleration, et. al.)
	RC Joystick data --> Holds user inputs to vehicle. Currently holds 4 channels of analog joystick data (currently
							commanding roll, pitch, yaw rate and either open or closed-loop height.)

	Propagates vehicle state, vehicle state vector and performs closed-loop control of attitude rates, attitude
	and height (depending on vehicle state, some of these parameters may be in open-loop control mode; please
	see the "vehicle_state_machine" driver software in the appropriate folder within "flight_software" for details
	on various initialization, flight-operational and failure modes.)

	Currently called from main() at precisely 500 Hz, via polling of gnc_update_flag using mission_timekeeper
	driver, operating off system-wide 1 kHz Systick ISR timebase.
 */

// static void vehicle_gnc_update(vehicle_state *st, imu_scaled_data_struct *imu_data, filtered_quadrotor_state *st_vector, vertical_state_kalman_struct *vertical_st_vect, rc_joystick_data_struct *user_joy_input)
// {
// 	float roll_cmd, pitch_cmd, yaw_cmd;
// 	float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;

// 	float user_height_command; // For open-loop height control, simply scaled joystick throttle axis input
// 	float height_controller_throttle_command; // For closed-loop height control

// 	#ifdef NESTED_HEIGHT_CONTROLLER
// 		float vertical_acceleration_command;
// 	#endif

// 	double motor_commands[4];

// 	// gpio_toggle(LBLUE2);
// 	/*
// 		Read Gyro, accelerometer and magnetometer sensors,
// 		and scale data to engineering (i.e. SI) units.

// 		Takes about 510 Microseconds to run, on 64 MHz system main clock,
// 		and using 400 KHz I2C bus clock along with SPI Clock Divisor of 64.

// 		Takes about 488 Microseconds to run, on 64 MHz system main clock,
// 		and using 400 KHz I2C bus clock along with SPI Clock Divisor of 32.
// 	 */
// 	get_scaled_imu_data(imu_data);

// 	/*
// 		Obtain filtered vehicle state through complementary filter.
// 		Takes about 25 Microseconds to run, on 64 MHz system main clock
// 	 */
// 	get_filtered_vehicle_state(st_vector, imu_data);

// 	/*
// 		EXPERIMENTAL: Fuse LIDAR-based height data with inertial (i.e. accelerometer data)
// 		using a Kalman filter
// 	 */
// 	// vertical_state_kalman_update(vertical_st_vect, *st_vector);

// 	/*
// 		Scale joystick vertical axis to user openloop vertical command:
// 	 */
// 	user_height_command = ((user_joy_input->vertical_channel_value)*0.5f)+0.5f;

// 	roll_cmd = 1.0f*user_joy_input->roll_channel_value;
// 	pitch_cmd = -1.0f*user_joy_input->pitch_channel_value;
// 	yaw_cmd = -1.0f*user_joy_input->yaw_channel_value;

// 	/*
// 		Based on vehicle state vector, user inputs via RC joystick and current state,
// 		update the state machine between the various initialization, flight and 
// 		emergency states:

// 		EXPERIMENTAL: THIS IS NOW BEING CALLED AFTER JOYSTICK VALUES ARE UPDATED ABOVE. 
// 						NEED TO PERFORM FLIGHT TEST TO MAKE SURE THIS IS A SAFE MODIFICATION!!!!
// 	 */
// 	propagate_vehicle_state_machine(st, *user_joy_input, *st_vector);

// 	#ifdef NESTED_HEIGHT_CONTROLLER
// 		if(get_flag_state(vertical_acceleration_control_update_flag) == STATE_PENDING)
// 		{
// 			reset_flag(vertical_acceleration_control_update_flag);
// 			generate_thrust_commands(st_vector->vertical_dynamic_acceleration_post_lpf, vertical_acceleration_command, &height_controller_throttle_command);	
// 		}
// 		if(get_flag_state(height_control_update_flag) == STATE_PENDING)
// 		{
// 			reset_flag(height_control_update_flag);
// 			generate_vertical_acceleration_commands(st_vector->height, user_height_command, &vertical_acceleration_command);
// 		}
// 	#endif

// 	#ifdef SINGLE_STAGE_HEIGHT_CONTROLLER
// 		if(get_flag_state(height_control_update_flag) == STATE_PENDING)
// 		{
// 			generate_thrust_commands(st_vector->height, user_height_command, &height_controller_throttle_command);
// 			reset_flag(height_control_update_flag);
// 		}
// 	#endif

// 	if(get_flag_state(angular_control_update_flag) == STATE_PENDING)
// 	{
// 		reset_flag(angular_control_update_flag);
// 		generate_rate_commands(st_vector, roll_cmd, pitch_cmd, yaw_cmd, &roll_rate_cmd, &pitch_rate_cmd, &yaw_rate_cmd);
// 		gpio_toggle(LBLUE2);
// 	}

// 	switch(*st)
// 	{
// 		case STATE_READY_FOR_TAKEOFF:
// 			rate_controller_update(motor_commands, imu_data, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, 0.0f);
// 			break;
// 		case STATE_MOTOR_RAMP_UP:
// 			rate_controller_update(motor_commands, imu_data, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, (float)get_throttle_command_ramp_variable());
// 			break;
// 		case STATE_CLOSEDLOOP_ATTITUDE:
// 			rate_controller_update(motor_commands, imu_data, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, user_height_command);
// 			break;
// 		case STATE_CLOSEDLOOP_ATTITUDE_HEIGHT:
// 			rate_controller_update(motor_commands, imu_data, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, height_controller_throttle_command);
// 			break;
// 		/*
// 			Verify telemetry input is still valid. Otherwise, shut all motors down.
// 			In future, this may evolve into more graceful behavior (return-to-home, et. al.)
// 		 */
// 		case STATE_TELEMETRY_SIGNAL_LOST:
// 			disable_controller();
// 			QuadRotor_motor1_stop();
// 			QuadRotor_motor2_stop();
// 			QuadRotor_motor3_stop();
// 			QuadRotor_motor4_stop();
// 			break;
// 		default:
// 			motor_commands[0] = 0.0f;
// 			motor_commands[1] = 0.0f;
// 			motor_commands[2] = 0.0f;
// 			motor_commands[3] = 0.0f;
// 	}
	
// 	#ifndef DISABLE_ALL_MOTORS
// 		QuadRotor_set_all_motors(motor_commands);
// 	#endif
// }

static float get_altitude_offset(lidar_lite_sensor_instance* ll_height_sensor_instance)
{
	uint8_t i = 0U;
	float running_sum = 0.0f;
	timekeeper_delay(35U);
	for(i = 0U; i < 100; ++i)
	{
		lidar_lite_trigger_measurement();
		timekeeper_delay(35U);
		running_sum += (float)lidar_lite_get_raw_data()/(float)100;
	}
	lidar_lite_initialize_instance(ll_height_sensor_instance);
	return running_sum/(float)100;
}

/*
	Get IMU data and complementary-filtered vehicle state estimate:
 */

static void get_vehicle_state(imu_scaled_data_struct *imu_data, 
								filtered_quadrotor_state *st_vector)
{
	/*
		Read Gyro, accelerometer and magnetometer sensors,
		and scale data to engineering (i.e. SI) units.

		Takes about 510 Microseconds to run, on 64 MHz system main clock,
		and using 400 KHz I2C bus clock along with SPI Clock Divisor of 64.

		Takes about 488 Microseconds to run, on 64 MHz system main clock,
		and using 400 KHz I2C bus clock along with SPI Clock Divisor of 32.
	 */
	get_scaled_imu_data(imu_data);

	/*
		Obtain filtered vehicle state through complementary filter.
		Takes about 25 Microseconds to run, on 64 MHz system main clock
	 */
	get_filtered_vehicle_state(st_vector, imu_data);
}

static void vehicle_stabilization_outerloop_update(filtered_quadrotor_state *st_vector,
													float roll_cmd_in,
													float pitch_cmd_in,
													float yaw_cmd_in,
													float *roll_rate_cmd_out,
													float *pitch_rate_cmd_out,
													float *yaw_rate_cmd_out)
{
	generate_rate_commands(st_vector, roll_cmd_in, pitch_cmd_in, yaw_cmd_in, roll_rate_cmd_out, pitch_rate_cmd_out, yaw_rate_cmd_out);
}

static void vehicle_stabilization_innerloop_update(imu_scaled_data_struct *imu_data,
													float roll_rate_cmd_in,
													float pitch_rate_cmd_in,
													float yaw_rate_cmd_in,
													float throttle_value_in,
													double *motor_commands_out)
{
	rate_controller_update(motor_commands_out, imu_data, roll_rate_cmd_in, pitch_rate_cmd_in, yaw_rate_cmd_in, throttle_value_in);
}

static void get_rc_joystick_data(rc_joystick_data_struct *js_data_in, float *roll_command_out,
																		float *pitch_command_out,
																		float *yaw_command_out,
																		float *height_command_out)
{
	get_rc_input_values(js_data_in);
	*height_command_out = ((js_data_in->vertical_channel_value)*0.5f)+0.5f;
	*roll_command_out = 1.0f*js_data_in->roll_channel_value;
	*pitch_command_out = -1.0f*js_data_in->pitch_channel_value;
	*yaw_command_out = -1.0f*js_data_in->yaw_channel_value;
}

int main(void)
{
	_disable_interrupts();

		imu_scaled_data_struct imu_struct_scaled;

		rc_joystick_data_struct js;
		vehicle_state qr_state_variable = STATE_IMU_CAL;

		set_system_clock();

		led_gpio_setup();
		systick_setup();
		usart_setup();

		QuadRotor_PWM_init();

		QuadRotor_motor1_setDuty(0.0f);
		QuadRotor_motor2_setDuty(0.0f);
		QuadRotor_motor3_setDuty(0.0f);
		QuadRotor_motor4_setDuty(0.0f);

		QuadRotor_motor1_start();
		QuadRotor_motor2_start();
		QuadRotor_motor3_start();
		QuadRotor_motor4_start();
		
		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

		printf("Initialized PWM channels\r\n");

#ifdef ENABLE_PWM_TEST_SHELL
		/*
			Simple shell for testing PWM functionality:
		 */

		pwm_test_shell();
#endif

		init_mission_timekeeper();
		initialize_imu(SCALE_2G, SCALE_1POINT9_GAUSS, SCALE_250_DPS, &imu_struct_scaled);

#ifdef ENABLE_LIDAR_LITE
		lidar_lite_sensor_instance ll_height_sensor_instance;
		lidar_lite_i2c_bus_setup();
		lidar_lite_initialize_instance(&ll_height_sensor_instance);
#endif

	_enable_interrupts();

	/*
		A small 500-millisecond delay to allow the LIDAR-Lite sensor to initialize and perform all
		self-test routines:
	 */
#ifdef ENABLE_LIDAR_LITE
	timekeeper_delay(500U);
#endif
#ifdef ENABLE_LIDAR_LITE
	float alt_offset = get_altitude_offset(&ll_height_sensor_instance);
#endif
#ifdef ENABLE_LIDAR_LITE
	uint8_t lidar_height_acquisition_flag = create_flag(LIDAR_LITE_MEASUREMENT_INTERVAL_MILLIS);
#endif
#ifdef NESTED_HEIGHT_CONTROLLER
		vertical_acceleration_control_update_flag = create_flag(10U);
#endif

	filtered_quadrotor_state st;
	do_bias_calculation(&imu_struct_scaled);
	init_comp_filter(&st);

	// vertical_state_kalman_struct vert_str;
	// vertical_state_kalman_init(&vert_str, 0.2f, 0.2f, 0.15f, 0.002f, 0.075f);

	// uint8_t state_estimation_flag = create_flag(1U);
	// uint8_t gnc_update_flag = create_flag(3U);
	// uint8_t rc_update_flag = create_flag(25U);
	// angular_control_update_flag = create_flag(4U);

	uint16_t state_estimation_interval_ms = (uint16_t)(DT_FILTER_LOOP*1000.0f);
	uint8_t state_estimation_flag = create_flag(state_estimation_interval_ms);

	uint16_t outerloop_control_interval_ms = (uint16_t)(ANGULAR_POSITION_CONTROL_DT*1000.0f);
	uint8_t outerloop_control_flag = create_flag(outerloop_control_interval_ms);

	uint16_t innerloop_control_interval_ms = (uint16_t)(ANGULAR_RATE_CONTROL_DT*1000.0f);
	uint8_t innerloop_control_flag = create_flag(innerloop_control_interval_ms);

	uint8_t rc_update_flag = create_flag(25U);

	init_rc_inputs(&js);

	controller_init_vars();
	// set_controller_mode(MODE_ANGULAR_POSITION_CONTROL);
	set_controller_mode(MODE_ANGULAR_RATE_CONTROL);
	enable_controller();

	printf("Ready for flight!!\r\n");

	initialize_vehicle_state_machine(&qr_state_variable);

	float roll_cmd, pitch_cmd, yaw_cmd, height_cmd;
	float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
	double motor_commands[4];

	roll_cmd = 0.0f;
	pitch_cmd = 0.0f;
	yaw_cmd = 0.0f;
	height_cmd = 0.0f;

	roll_rate_cmd = 0.0f;
	pitch_rate_cmd = 0.0f;
	yaw_rate_cmd = 0.0f;

	motor_commands[0] = 0.0f;
	motor_commands[1] = 0.0f;
	motor_commands[2] = 0.0f;
	motor_commands[3] = 0.0f;

	while (1)
	{

		if(get_flag_state(state_estimation_flag) == STATE_PENDING)
		{
			reset_flag(state_estimation_flag);
			get_vehicle_state(&imu_struct_scaled, &st);

			gpio_toggle(LBLUE2);
			
			if(get_flag_state(outerloop_control_flag) == STATE_PENDING)
			{
				reset_flag(outerloop_control_flag);
				vehicle_stabilization_outerloop_update(&st, roll_cmd, pitch_cmd, yaw_cmd,
															&roll_rate_cmd, &pitch_rate_cmd, &yaw_rate_cmd);
				// gpio_toggle(LBLUE2);
			}

			if(get_flag_state(innerloop_control_flag) == STATE_PENDING)
			{
				reset_flag(innerloop_control_flag);
				vehicle_stabilization_innerloop_update(&imu_struct_scaled, roll_rate_cmd, 
																			pitch_rate_cmd, 
																			yaw_rate_cmd,
																			height_cmd,
																			motor_commands);
				QuadRotor_set_all_motors(motor_commands);
				// gpio_toggle(LBLUE2);
			}

			if(get_flag_state(rc_update_flag) == STATE_PENDING)
			{
				reset_flag(rc_update_flag);
				get_rc_joystick_data(&js, &roll_cmd, &pitch_cmd, &yaw_cmd, &height_cmd);

				if(js.vertical_channel_validity == CHANNEL_INVALID || js.roll_channel_validity == CHANNEL_INVALID ||
					js.pitch_channel_validity == CHANNEL_INVALID || js.yaw_channel_validity == CHANNEL_INVALID)
				{
					disable_controller();
					QuadRotor_motor1_stop();
					QuadRotor_motor2_stop();
					QuadRotor_motor3_stop();
					QuadRotor_motor4_stop();
				}
			}
		}

// 		if(get_flag_state(gnc_update_flag) == STATE_PENDING)
// 		{
// 			reset_flag(gnc_update_flag);
// 			vehicle_gnc_update(&qr_state_variable, &imu_struct_scaled, &st, &vert_str, &js);
// 		}

// 		if(get_flag_state(rc_update_flag) == STATE_PENDING)
// 		{
// 			reset_flag(rc_update_flag);
// 			get_rc_input_values(&js);
// 		}
// #ifdef ENABLE_LIDAR_LITE
// 		if(get_flag_state(lidar_height_acquisition_flag) == STATE_PENDING)
// 		{
// 			reset_flag(lidar_height_acquisition_flag);
// 			lidar_lite_propagate_state_machine(&ll_height_sensor_instance);
// 			// propagate_compensated_vehicle_height(lidar_lite_get_latest_measurement(ll_height_sensor_instance) 
// 			// 											- alt_offset,
// 			// 										&st);
// 			propagate_compensated_vehicle_height(lidar_lite_get_latest_measurement(ll_height_sensor_instance) 
// 														- alt_offset,
// 														0.0535f,
// 														0.0585f,
// 														&st);
// 			/*
// 				If offset-compensated height is negative, we limit it to zero, since negative relative
// 				altitude is physically impossible, and indicative of just a sensor glitch/drift in offset:
// 			 */
// 			if(st.height < 0.0f)
// 			{
// 				st.height = 0.0f;
// 			}
// 		}
// #endif
	}
	return 0;
}
