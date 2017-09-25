#include "consoleIO.h"

static void dbg_monitor(int argc, char** argv)
{
  printf("Beginning debug log display.\r\n");
  printf("Press any key to continue, or press \'x\' to exit.\r\n");
  char ch = ' ';
  while(ch != 'x' && ch != 'X')
  {
    scanf("%c", &ch);
    dbgIPCPopFromQueue();
  }
}

void setup_consoleIO(void)
{
  if(dbgIPCInit() < 0)
  {
    while(1);
  }
  xTaskCreate(
      consoleIOTask,                    	/* Function pointer */
      "consoleIOTask",                      /* Task name - for debugging only*/
      configMINIMAL_STACK_SIZE,         	/* Stack depth in words */
      (void*) NULL,                     	/* Pointer to tasks arguments (parameter) */
      tskIDLE_PRIORITY + consoleProc_PRIO,  /* Task priority*/
      NULL                              	/* Task handle */
  );
}

void consoleIOTask(void *pvParameters)
{
  setup_system_shell();
  if(install_cmd("dbgmon", dbg_monitor) < 0)
  {
    printf("ERROR installing command dbgmon!!\r\n");
  }
  while(1)
  {
    shell_run();
    vTaskDelay(pdMS_TO_TICKS(consoleProc_pdMS));
  }
}