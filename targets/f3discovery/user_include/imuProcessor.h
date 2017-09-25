#ifndef IMU_PROCESSOR_H_
#define IMU_PROCESSOR_H_	1

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "imu_hal.h"

#define imuProc_PRIO	3UL
#define imuProc_pdMS	2

void setup_imuProcessor(void);
void imuProcessorTask(void *pvParameters);

extern imu_desc mpu9250_spi, lsm303_i2c;

#endif