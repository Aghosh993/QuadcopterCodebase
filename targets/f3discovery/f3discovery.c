/*
  This is the main user software application
  This application sets up the hardware and FreeRTOS kernel, and then
  branches to the FreeRTOS scheduler.

  Tasks:

  consoleIOTask: Provides a console over UART (UART1 via PC4/5 to 
                  the STLink/Black Magic Probe Virtual COM Port lines)
                  Baud = 115200, 8n1
                  The routing of the UART can be changed by using -DFTDI_SERIAL
                  instead of the default -DBLACKMAGIC_PROBE_SERIAL (see Makefile)

  (c) Abhimanyu Ghosh, 2017
 */

/*
  Standard includes:
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*
  FreeRTOS headers:
 */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*
  BSP (Board Support Package) headers:
 */
#include "serial_comms_highlevel.h"
#include "cpu_hal_interface.h"
#include "board_led.h"
#include "sys_timer.h"
#include "imu_hal.h"

/*
  Userspace applications/processes:
 */
#include "imuProcessor.h"
#include "consoleIO.h"
#include "pwm_input.h"
#include "vehicleMixer.h"

/*
  Userspace libraries that are RTOS-independent:
 */
#include "rt_telemetry.h"
#include "complementary_filter.h"

serialport debug_uart1;
volatile rt_telemetry_comm_channel telem0;

volatile float vehicleState[3];

extern imu_desc l3gd20_spi, mpu9250_spi;

/*
  Verified that STM32 Cube HAL configures SysTick to be lowest priority ISR (0xF)
  at NVIC level, so using this for the RTOS tick should be okay...
 */
void SysTick_Handler(void) {
    /*
      Run the FreeRTOS scheduler:
     */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }

    /*
      Call ST's timekeeping functions:
     */
    HAL_IncTick();

    /*
      Increment our own timebase:
     */
    sys_timerUpdate();
}

void SysInitTask(void *pvParameters)
{
  setup_pwmInputProcessor(false);
  while(!pwmInputReady());

  vTaskDelay(pdMS_TO_TICKS(500));

  esc_init();
  esc1_start();
  esc2_start();
  esc3_start();
  esc4_start();

  initAltCtlLaw();
  setAltCtrlMode_stabilizer();

  rc_joystick_data_struct js0;

  while(1)
  {
    if(pwmReadyForNewCmd())
    {
      getPwmInput(&js0);
      updateUserCmds(js0);
      readIMU();
      calcBodyRateCmds();
      bodyRateLoop();
      send_telem_msg_n_floats_blocking(&telem0, "test", 4, vehicleState, 3);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

int main()
{
  /*
    Initialize the PLL, clock tree to all peripherals, flash and Systick 1 ms time reference:
   */
  cpu_init();
  sys_timerInit();

  serialport_init(&debug_uart1, PORT1);
  rt_telemetry_init_channel(&telem0, &debug_uart1);

  /*
    Disable buffered I/O for UART comms:
  */
  // setvbuf(stdin,NULL,_IONBF,0);   // Sets stdin in unbuffered mode (normal for usart com)
  // setvbuf(stdout,NULL,_IONBF,0);  // Sets stdin in unbuffered mode (normal for usart com)
  // printf("\r\n");                 // Go to new line for readability
  /*
    Initialize the GPIO (General-Purpose I/O) subsystem pins that are connected to the LEDs on the board:
   */
  board_led_init();
  board_button_init();

  int ret = imu_hal_init(&mpu9250_spi, SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS);
  if(ret<0)
  {
    // printf("ERROR in IMU1 init, errcode %d!!\r\n", ret);
    while(1);
  }

  // ret = imu_hal_init(&lsm303_i2c, SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS);
  // if(ret<0)
  // {
  //   printf("ERROR in IMU2 init, errcode %d!!\r\n", ret);
  //   while(1);
  // }

  ret = imu_hal_init(&l3gd20_spi, SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS);
  if(ret<0)
  {
    // printf("ERROR in IMU3 init, errcode %d!!\r\n", ret);
    while(1);
  }

  xTaskCreate(
      SysInitTask,                     /* Function pointer */
      "SysInitTask",                   /* Task name - for debugging only*/
      configMINIMAL_STACK_SIZE,         /* Stack depth in words */
      (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
      tskIDLE_PRIORITY + 2UL,           /* Task priority*/
      NULL                              /* Task handle */
  );
  
  // printf("FreeRTOS Kernel and system HAL init finished, proceeding to scheduler startup...\r\n");
  vTaskStartScheduler();

  /*
    Do nothing here... the FreeRTOS kernel should be running and executing all scheduled tasks...
   */
  while(1);

  return 0;
}
