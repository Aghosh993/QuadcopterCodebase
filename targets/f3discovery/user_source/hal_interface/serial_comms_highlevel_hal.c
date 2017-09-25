/*
	File: serial_comms_highlevel_hal.c

	(c) Abhimanyu Ghosh, 2016
 */

#include "serial_comms_highlevel_hal.h"
#include "serial_comms_highlevel.h"

static inline void _enableIRQ(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case PORT1:
			HAL_NVIC_EnableIRQ(USART1_IRQn);
			break;
		case PORT2:
			HAL_NVIC_EnableIRQ(USART2_IRQn);
			break;
		default:
			break;
	}
}

static inline void _disableIRQ(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case PORT1:
			HAL_NVIC_DisableIRQ(USART1_IRQn);
			break;
		case PORT2:
			HAL_NVIC_DisableIRQ(USART2_IRQn);
			break;
		default:
			break;
	}
}

static volatile UART_HandleTypeDef UartHandle1;
// static volatile UART_HandleTypeDef UartHandle2;

extern serialport debug_uart1, pxflow_gps_uart;

static void UART1_Init(void)
{
	UartHandle1.Instance        = USART1;

	UartHandle1.Init.BaudRate     = 115200;
	UartHandle1.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle1.Init.StopBits     = UART_STOPBITS_1;
	UartHandle1.Init.Parity       = UART_PARITY_NONE;
	UartHandle1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle1.Init.Mode         = UART_MODE_TX_RX;
	UartHandle1.Init.OverSampling = UART_OVERSAMPLING_16;
	UartHandle1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	HAL_UART_DeInit(&UartHandle1);

	if(HAL_UART_Init(&UartHandle1) != HAL_OK)
	{
		while(1);
	}
  	__HAL_UART_ENABLE_IT(&UartHandle1, USART_IT_RXNE);  
	HAL_NVIC_SetPriority(USART1_IRQn, 1, 3);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

// static void UART2_Init(void)
// {
// 	UartHandle2.Instance        = USART2;

// 	UartHandle2.Init.BaudRate     = 115200;
// 	UartHandle2.Init.WordLength   = UART_WORDLENGTH_8B;
// 	UartHandle2.Init.StopBits     = UART_STOPBITS_1;
// 	UartHandle2.Init.Parity       = UART_PARITY_NONE;
// 	UartHandle2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
// 	UartHandle2.Init.Mode         = UART_MODE_TX_RX;
// 	UartHandle2.Init.OverSampling = UART_OVERSAMPLING_16;
// 	UartHandle2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

// 	HAL_UART_DeInit(&UartHandle2);

// 	if(HAL_UART_Init(&UartHandle2) != HAL_OK)
// 	{
// 		while(1);
// 	}

//   __HAL_UART_ENABLE_IT(&UartHandle2, USART_IT_RXNE);
// 	HAL_NVIC_SetPriority(USART2_IRQn, 1, 2);
// 	HAL_NVIC_EnableIRQ(USART2_IRQn);
// }

void USART1_IRQHandler(void)
{
  char c;

  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_WUF))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_WUF);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_CM))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_CM);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_CTS))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_CTS);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_LBD))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_LBD);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_TC))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_TC);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_IDLE))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_IDLE);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_ORE))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_ORE);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_NE))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_NE);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_FE))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_FE);    
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_PE))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_PE);    
  }

  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_TXE))
  {
  	__HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_TXE);    
  	serialport_highlevel_tx_isr(&debug_uart1);
  }
  if(__HAL_UART_GET_IT(&UartHandle1, UART_IT_RXNE))
  {
    __HAL_UART_CLEAR_IT(&UartHandle1, UART_IT_RXNE);
    serialport_highlevel_rx_isr(&debug_uart1);
  }
}

// void USART2_IRQHandler(void)
// {
//   char c;

//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_WUF))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_WUF);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_CM))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_CM);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_CTS))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_CTS);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_LBD))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_LBD);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_TC))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_TC);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_IDLE))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_IDLE);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_ORE))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_ORE);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_NE))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_NE);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_FE))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_FE);    
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_PE))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_PE);    
//   }

//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_TXE))
//   {
//   	__HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_TXE);
//   	serialport_highlevel_tx_isr(&pxflow_gps_uart);  
//   }
//   if(__HAL_UART_GET_IT(&UartHandle2, UART_IT_RXNE))
//   {
//     __HAL_UART_CLEAR_IT(&UartHandle2, UART_IT_RXNE);
//     serialport_highlevel_rx_isr(&pxflow_gps_uart);
//   }
// }

/*
	Initializes all required high-level real/virtual serial port HAL drivers:
 */
void serialport_hal_init(serialport_desc port_descriptor)
{
  _disableIRQ(port_descriptor);
	switch(port_descriptor)
	{
		case PORT1:
			UART1_Init();
			break;
		case PORT2:
			// UART2_Init();
			break;
	}
  _enableIRQ(port_descriptor);
}

void serialport_hal_enable_tx_isr(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case PORT1:
			__HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_TXE);
			break;
		case PORT2:
			// __HAL_UART_ENABLE_IT(&UartHandle2, UART_IT_TXE);
			break;
		default:
			break;
	}
}

void serialport_hal_disable_tx_isr(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case PORT1:
			__HAL_UART_DISABLE_IT(&UartHandle1, UART_IT_TXE);
			break;
		case PORT2:
			// __HAL_UART_DISABLE_IT(&UartHandle2, UART_IT_TXE);
			break;
		default:
			break;
	}
}

int serialport_send_byte(serialport_desc port_descriptor, uint8_t byte_to_send)
{
	switch(port_descriptor)
	{
		case PORT1:
			while((__HAL_UART_GET_FLAG(&UartHandle1, UART_FLAG_TXE) ? SET : RESET) == RESET);
			UartHandle1.Instance->TDR = (uint16_t)byte_to_send;
			return 0;
		case PORT2:
			// while((__HAL_UART_GET_FLAG(&UartHandle2, UART_FLAG_TXE) ? SET : RESET) == RESET);
			// UartHandle2.Instance->TDR = (uint16_t)byte_to_send;
			return 0;
		default:
			return -1;
	}
}

uint8_t serialport_receive_byte(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case PORT1:
			while((__HAL_UART_GET_FLAG(&UartHandle1, UART_FLAG_RXNE) ? SET : RESET) == RESET);
			return UartHandle1.Instance->RDR;
		case PORT2:
			// while((__HAL_UART_GET_FLAG(&UartHandle2, UART_FLAG_RXNE) ? SET : RESET) == RESET);
			// return UartHandle2.Instance->RDR;
		default:
			return 0;
	}
}