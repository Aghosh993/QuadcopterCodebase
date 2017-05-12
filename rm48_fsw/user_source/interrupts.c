/*
	File: interrupts.c

	(c) Abhimanyu Ghosh, 2016
 */

#include "interrupts.h"
#include "Quadcopter_PWM_HAL.h"
#include "vehicle_gnc.h"

extern serialport ftdi_dbg_port;

extern float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
extern float throttle_value_common;

extern uint8_t gnc_innerloop_flag;

void rti_callback(void)
{
	flag_scheduler_callback();
	update_mission_time_counter();
}

void sci1_tx_callback(void)
{
	serialport_highlevel_tx_isr(&ftdi_dbg_port);
}

void sci1_rx_callback(void)
{
	#ifdef SCI1_LOOPBACK
		uint8_t data = sciReceiveByte(sciREG);
		sciSendByte(sciREG, data);
	#endif
	#ifndef SCI1_LOOPBACK
		serialport_highlevel_rx_isr(&ftdi_dbg_port);
	#endif
}

void sci2_tx_callback(void)
{
	// serialport_highlevel_tx_isr(tm4c_comms_aux_port_ptr);
}

void sci2_rx_callback(void)
{
	#ifdef SCI2_LOOPBACK
		uint8_t data = sciReceiveByte(scilinREG);
		sciSendByte(scilinREG, data);
	#endif
	#ifndef SCI2_LOOPBACK
		// serialport_highlevel_rx_isr(tm4c_comms_aux_port_ptr);
	#endif
}

// extern float m1_cmd, m2_cmd, m3_cmd, m4_cmd;
extern volatile uint8_t update_cmds;

void pwm_callback(hetBASE_t* hetREG, uint32 pwm)
{
	// double motor_output_commands[4];
	
	// motor_output_commands[0] = 0.0f;
	// motor_output_commands[1] = 0.0f;
	// motor_output_commands[2] = 0.0f;
	// motor_output_commands[3] = 0.0f;

	if(hetREG == hetREG1 && pwm == pwm0)
	{
		/*
			Run GNC inner loop to generate 4 motor PWM commands:
		 */
		// if(gnc_enabled())
		// {
			// gnc_vehicle_stabilization_innerloop_update(roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd,
			// 											throttle_value_common,
			// 											motor_output_commands);
			// QuadRotor_motor1_setDuty((float)motor_output_commands[0]);                             
			// QuadRotor_motor2_setDuty((float)motor_output_commands[1]);
			// QuadRotor_motor3_setDuty((float)motor_output_commands[2]);
			// QuadRotor_motor4_setDuty((float)motor_output_commands[3]);
		// }
		// QuadRotor_motor1_setDuty(m1_cmd);
		// QuadRotor_motor2_setDuty(m2_cmd);
		// QuadRotor_motor3_setDuty(m3_cmd);
		// QuadRotor_motor4_setDuty(m4_cmd);
		update_cmds = 1;
	}
}