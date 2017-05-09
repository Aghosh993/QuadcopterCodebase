#ifndef TELEM_CONFIG_H_
#define TELEM_CONFIG_H_	1

// The defines below control telemetry over UART (i.e. via TRTP prototocol):

// #define SEND_RC_INPUT_TELEMETRY			1
// #define SEND_MPU_IMU_TELEMETRY			1
// #define SEND_MOTOR_CMDS_TELEMETRY		1
// #define SEND_STATE_VECTOR_TELEMETRY		1
// #define SEND_PWM_INPUT_TELEMETRY			1
// #define SEND_HEIGHT_SENSOR_TELEMETRY		1 //??
// #define SEND_HEIGHT_ESTIMATOR_TELEMETRY		1
// #define SEND_HEADING_TELEMETRY				1
// #define SEND_BNO_SERIAL_STATS_TELEMETRY		1

/////////////////////////////////////////////////////////////////////////////

// The defines below control telemetry emitted over the CAN bus:

#define SEND_CAN_ROLL_PITCH					1
#define SEND_CAN_YAW_HEIGHT					1
#define SEND_CAN_VERT_VEL					1

#endif