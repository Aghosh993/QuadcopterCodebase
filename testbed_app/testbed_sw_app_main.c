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
#include <level0_comms.h>

/*
	Shamelessly stolen from I2C example in libopencm3-examples,
	but sharing is caring, right? Right? Okay.
 */

// #define LRED GPIOE, GPIO9
// #define LORANGE GPIOE, GPIO10
// #define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
// #define LRED2 GPIOE, GPIO13
// #define LORANGE2 GPIOE, GPIO14
// #define LGREEN2 GPIOE, GPIO15

/*
	Approx active LOC (Lines of code: ~3200)
 */

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA2,3 			-> 			USART2
	PE8,9,10,11,12,13,14,15 -> 	LEDs
	PB6,7 			-> 			I2C
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
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

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
	float pwm_setval = 0.0f;
	char cmd;
	uint8_t err = 0U;
	// timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MAX_VAL+100);
	// timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MAX_VAL+100);
	// timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MAX_VAL+100);
	// timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MAX_VAL+100);
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
			// case 'A':
			// timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MIN_VAL-100);
			// timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MIN_VAL-100);
			// timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MIN_VAL-100);
			// timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MIN_VAL-100);
			// err=0;
			// break;
			// case 'B':
			// timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MAX_VAL+100);
			// timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MAX_VAL+100);
			// timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MAX_VAL+100);
			// timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MAX_VAL+100);
			// err=0;
			// break;
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

static volatile uint8_t angular_control_update_flag;
static volatile uint8_t height_control_update_flag;

static void vehicle_gnc_update(imu_scaled_data_struct *imu_data, filtered_quadrotor_state *st_vector, rc_joystick_data_struct *user_joy_input)
{
	gpio_toggle(LBLUE2);
	float roll_cmd, pitch_cmd, yaw_cmd;
	float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;

	float user_height_command;

	double motor_commands[4];

	// gpio_toggle(LBLUE2);
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

	if(user_joy_input->vertical_channel_validity == CHANNEL_INVALID || user_joy_input->roll_channel_validity == CHANNEL_INVALID ||
		user_joy_input->pitch_channel_validity == CHANNEL_INVALID || user_joy_input->yaw_channel_validity == CHANNEL_INVALID)
	{
		disable_controller();
		QuadRotor_motor1_stop();
		QuadRotor_motor2_stop();
		QuadRotor_motor3_stop();
		QuadRotor_motor4_stop();
	}

	/*
		Scale joystick vertical axis to user openloop vertical command:
	 */
	user_height_command = ((user_joy_input->vertical_channel_value)*0.5f)+0.5f;

	roll_cmd = 1.0f*user_joy_input->roll_channel_value;
	pitch_cmd = -1.0f*user_joy_input->pitch_channel_value;
	yaw_cmd = -1.0f*user_joy_input->yaw_channel_value;

	if(get_flag_state(angular_control_update_flag) == STATE_PENDING)
	{
		reset_flag(angular_control_update_flag);
		// generate_rate_commands(st_vector, roll_cmd, pitch_cmd, yaw_cmd, &roll_rate_cmd, &pitch_rate_cmd, &yaw_rate_cmd);
		// gpio_toggle(LBLUE2);
	}

	// rate_controller_update(motor_commands, imu_data, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, user_height_command);
	
	// QuadRotor_set_all_motors(motor_commands);
}

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

float kalman_roll = 0.0f;

void kalman_estimate(imu_scaled_data_struct *imu_data, filtered_quadrotor_state *st)
{

}

int main(void)
{
	imu_i2c_bus_clear();
	_disable_interrupts();

		imu_scaled_data_struct imu_struct_scaled;
		imu_raw_data_struct imu_struct_raw;

		rc_joystick_data_struct js;

		set_system_clock();

		led_gpio_setup();
		systick_setup();
		usart_setup();

		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

		init_mission_timekeeper();
		initialize_imu(SCALE_2G, SCALE_1POINT9_GAUSS, SCALE_250_DPS, &imu_struct_scaled);

		// lidar_lite_sensor_instance ll_height_sensor_instance;
		// lidar_lite_i2c_bus_setup();
		// lidar_lite_initialize_instance(&ll_height_sensor_instance);
		
	_enable_interrupts();

	timekeeper_delay(500U);
	// float alt_offset = get_altitude_offset(&ll_height_sensor_instance);

	filtered_quadrotor_state st;
	do_bias_calculation(&imu_struct_scaled);
	// init_comp_filter(&st);

	// uint8_t gnc_update_flag = create_flag(2U);
	// uint8_t telemetry_flag = create_flag(50U);
	// uint8_t rc_update_flag = create_flag(25U);
	// uint8_t lidar_height_acquisition_flag = create_flag(LIDAR_LITE_MEASUREMENT_INTERVAL_MILLIS);

	// angular_control_update_flag = create_flag(4U);
	// height_control_update_flag = create_flag(25U);

	#ifdef ENABLE_PWM_TEST_SHELL

			/*
				Simple shell for testing PWM functionality:
			 */
			_disable_interrupts();

			QuadRotor_PWM_init();
			QuadRotor_motor1_start();
			QuadRotor_motor2_start();
			QuadRotor_motor3_start();
			QuadRotor_motor4_start();
			
			_enable_interrupts();

			pwm_test_shell();

	#endif

	// init_rc_inputs(&js);

	// controller_init_vars();
	// set_controller_mode(MODE_ANGULAR_POSITION_CONTROL);

	// enable_controller();

	// _disable_interrupts();

	// 	QuadRotor_PWM_init();
	// 	QuadRotor_motor1_start();
	// 	QuadRotor_motor2_start();
	// 	QuadRotor_motor3_start();
	// 	QuadRotor_motor4_start();
	// 	printf("Initialized PWM channels\r\n");

	// _enable_interrupts();

	printf("Ready\r\n");

	uint8_t user_output_flag = create_flag(100U);
	uint8_t kalman_estimate_flag = create_flag(1U);

	float corrected_gyro_data[3];

	binary_communications_manager uart2_cm;
	init_comm_lib(&uart2_cm, send_byte_to_gs);

	float yaw_non_comp;
	/*
	X off = 0.0057682278876427058 Y off = -0.075884129230699135 Z off = 0.034063366379292637 R = 0.46355354507479524
	*/

	float x_off = 0.0057682278876427058f;
	float y_off = -0.075884129230699135f;
	float z_off = 0.034063366379292637f;

	while (1)
	{
		if(get_flag_state(user_output_flag) == STATE_PENDING)
		{
			reset_flag(user_output_flag);
			get_scaled_imu_data(&imu_struct_scaled);
			get_corrected_scaled_gyro_data(&imu_struct_scaled, corrected_gyro_data);
			// printf("Roll rate: %f Pitch rate: %f\r\n", corrected_gyro_data[AXIS_ROLL], corrected_gyro_data[AXIS_PITCH]);
			// printf("Hello!!\r\n");
			// printf("MagX: %f MagY: %f MagZ: %f\r\n", imu_struct_scaled.magnetometer_data[AXIS_X],
			// 											imu_struct_scaled.magnetometer_data[AXIS_Y],
			// 											imu_struct_scaled.magnetometer_data[AXIS_Z]);

			send_float_triple(&uart2_cm, imu_struct_scaled.magnetometer_data[AXIS_X],
											imu_struct_scaled.magnetometer_data[AXIS_Y],
											imu_struct_scaled.magnetometer_data[AXIS_Z]);

			// From http://diydrones.com/forum/topics/heading-from-3d-magnetometer:

			// yaw_non_comp = 57.295827909f*atan2f(-1.0f*(imu_struct_scaled.magnetometer_data[AXIS_Y]-x_off),
			// 							(imu_struct_scaled.magnetometer_data[AXIS_X]-y_off));
			// printf("%f\r\n", yaw_non_comp);
		}
		// if(get_flag_state(gnc_update_flag) == STATE_PENDING)
		// {
		// 	reset_flag(gnc_update_flag);
		// 	vehicle_gnc_update(&imu_struct_scaled, &st, &js);
		// }
		// if(get_flag_state(telemetry_flag) == STATE_PENDING)
		// {
		// 	// printf("Height in meters: %f\r\n", st.height);
		// 	// write_i2c(I2C2, 0x62, 0x00, 1, &send_byte);
		// 	// timekeeper_delay(40U);
		// 	// read_i2c(I2C2, 0x62, 0x8f, 2, height_data_cm);
		// 	// printf("Height in cm: %d\r\n", height_data_cm[0]<<8 | height_data_cm[1]);
		// 	// reset_flag(telemetry_flag);
		// 	// get_scaled_imu_data(&imu_struct_scaled);
		// 	// printf("%f, %f, %f\r\n", imu_struct_scaled.magnetometer_data[AXIS_X], imu_struct_scaled.magnetometer_data[AXIS_Y], imu_struct_scaled.magnetometer_data[AXIS_Z]);
		// 	// printf("%d, %d, %d\r\n", imu_struct_raw.magnetometer_data[AXIS_X], imu_struct_raw.magnetometer_data[AXIS_Y], imu_struct_raw.magnetometer_data[AXIS_Z]);
		// 	// printf("Roll: %f Pitch: %f Yaw: %f\r\n", st.roll, st.pitch, st.yaw);
		// 	// get_scaled_imu_data(&imu_struct_scaled);
		// 	// printf("Roll rate: %f Pitch rate: %f\r\n", imu_struct_scaled.gyro_data[AXIS_ROLL], imu_struct_scaled.gyro_data[AXIS_PITCH]);
		// }
		// if(get_flag_state(rc_update_flag) == STATE_PENDING)
		// {
		// 	reset_flag(rc_update_flag);
		// 	// get_rc_input_values(&js);
		// }

		// if(get_flag_state(lidar_height_acquisition_flag) == STATE_PENDING)
		// {
		// 	// reset_flag(lidar_height_acquisition_flag);
		// 	// lidar_lite_propagate_state_machine(&ll_height_sensor_instance);
		// 	// st.height = lidar_lite_get_latest_measurement(ll_height_sensor_instance);
		// 	reset_flag(lidar_height_acquisition_flag);
		// 	lidar_lite_propagate_state_machine(&ll_height_sensor_instance);
		// 	propagate_compensated_vehicle_height(lidar_lite_get_latest_measurement(ll_height_sensor_instance) 
		// 												- alt_offset, 0.0f, 0.0f,
		// 											&st);

		// 	/*
		// 		If offset-compensated height is negative, we limit it to zero, since negative relative
		// 		altitude is physically impossible, and indicative of just a sensor glitch/drift in offset:
		// 	 */
		// 	if(st.height < 0.0f)
		// 	{
		// 		st.height = 0.0f;
		// 	}
		// }

		// if(js.vertical_channel_validity != CHANNEL_INVALID && js.roll_channel_validity != CHANNEL_INVALID &&
		// 	js.pitch_channel_validity != CHANNEL_INVALID && js.yaw_channel_validity != CHANNEL_INVALID)
		// {
		// 	printf("Roll: %f Pitch: %f Yaw: %f Vertical: %f\r\n", js.roll_channel_value, js.pitch_channel_value,
		// 															js.yaw_channel_value, js.vertical_channel_value);
		// }
		// else
		// {
		// 	printf("Signal lost\r\n");
		// }

		// printf("Ax: %f Ay: %f Az: %f\r\n", istr_s.accel_data[AXIS_X], istr_s.accel_data[AXIS_Y], 
		// 									istr_s.accel_data[AXIS_Z]);
		// printf("Mx: %f My: %f Mz: %f\r\n", istr_s.magnetometer_data[AXIS_X], istr_s.magnetometer_data[AXIS_Y], 
		// 									istr_s.magnetometer_data[AXIS_Z]);
		// printf("Gx: %f Gy: %f Gz: %f\r\n", istr_s.gyro_data[AXIS_X], istr_s.gyro_data[AXIS_Y], 
		// 									istr_s.gyro_data[AXIS_Z]);
	}

	return 0;
}
