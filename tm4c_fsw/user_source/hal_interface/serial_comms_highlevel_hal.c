/*
	File: serial_comms_highlevel_hal.c

	(c) Abhimanyu Ghosh, 2016
 */

#include "serial_comms_highlevel_hal.h"

static void can0_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock GPIO_CR register with this magic value
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0xFF;

    GPIOPinConfigure(GPIO_PF0_CAN0RX);
    GPIOPinConfigure(GPIO_PF3_CAN0TX); 
    GPIOPinTypeCAN(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));

    CANInit(CAN0_BASE);

    if(CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000)!=500000)
    {
        while(1);
    }

    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    
    CANEnable(CAN0_BASE);

    tCANMsgObject sMsgObjectRx_uart5;
    sMsgObjectRx_uart5.ui32MsgID = 0xC0000002;
    sMsgObjectRx_uart5.ui32MsgIDMask = 0xFFFF;
    sMsgObjectRx_uart5.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx_uart5.ui32MsgLen = 1;
    CANMessageSet(CAN0_BASE, 4, &sMsgObjectRx_uart5, MSG_OBJ_TYPE_RX);

    tCANMsgObject sMsgObjectRx_uart6;
    sMsgObjectRx_uart6.ui32MsgID = 0xC0000004;
    sMsgObjectRx_uart6.ui32MsgIDMask = 0xFFFF;
    sMsgObjectRx_uart6.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx_uart6.ui32MsgLen = 1;
    CANMessageSet(CAN0_BASE, 5, &sMsgObjectRx_uart6, MSG_OBJ_TYPE_RX);

    tCANMsgObject sMsgObjectRx_uart7;
    sMsgObjectRx_uart7.ui32MsgID = 0xC0000006;
    sMsgObjectRx_uart7.ui32MsgIDMask = 0xFFFF;
    sMsgObjectRx_uart7.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx_uart7.ui32MsgLen = 1;
    CANMessageSet(CAN0_BASE, 6, &sMsgObjectRx_uart7, MSG_OBJ_TYPE_RX);

    CANRetrySet(CAN0_BASE, true);
}

static void can0_uart_relay(serialport_desc u, uint8_t byte_to_send)
{
    // UARTCharPut(UART6_BASE, byte_to_send);
    uint8_t data[8];
    tCANMsgObject sCANMessage;
    
    data[0] = byte_to_send;

    sCANMessage.ui32MsgIDMask = 0xF; // Do we need this for TX objects?? We'll find out :)
    sCANMessage.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    sCANMessage.ui32MsgLen = 1;
    sCANMessage.pui8MsgData = &data[0];

    switch(u)
    {
        case CAN_ENCAP_UART5:
            sCANMessage.ui32MsgID = 0x60000001;
            CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);
            break;
        case CAN_ENCAP_UART6:
            sCANMessage.ui32MsgID = 0x60000003;
            CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_TX);
            break;
        case CAN_ENCAP_UART7:
            sCANMessage.ui32MsgID = 0x60000005;
            CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_TX);
            break;
        default:
            break;
    }
}

static void uart5_setup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));

    GPIOPinConfigure(GPIO_PJ2_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);

    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART5));

    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_TX);
    UARTTxIntModeSet(UART5_BASE, UART_TXINT_MODE_EOT);
    IntEnable(INT_UART5);
    UARTEnable(UART5_BASE);
    UARTFIFODisable(UART5_BASE);
}

static void uart6_setup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    GPIOPinConfigure(GPIO_PD4_U6RX);
    GPIOPinConfigure(GPIO_PD5_U6TX);

    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));

    UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_TX);
    UARTTxIntModeSet(UART6_BASE, UART_TXINT_MODE_EOT);
    IntEnable(INT_UART6);
    UARTEnable(UART6_BASE);
    UARTFIFODisable(UART6_BASE);
}

static void uart7_setup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinConfigure(GPIO_PE1_U7TX);

    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7));

    UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_TX);
    UARTTxIntModeSet(UART7_BASE, UART_TXINT_MODE_EOT);
    IntEnable(INT_UART7);
    UARTEnable(UART7_BASE);
    UARTFIFODisable(UART7_BASE);
}

/*
	Initializes all required high-level real/virtual serial port HAL drivers:
 */
void serialport_hal_init(void)
{
	// can0_init();
 //    uart5_setup();
 //    uart6_setup();
    uart7_setup();
}

void serialport_hal_enable_tx_isr(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case UART5:
			UARTIntEnable(UART5_BASE, UART_INT_TX);
			break;
		case UART6:
			UARTIntEnable(UART6_BASE, UART_INT_TX);
			break;
		case UART7:
            UARTIntEnable(UART7_BASE, UART_INT_TX);
			break;
		case CAN_ENCAP_UART5:
			break;
		case CAN_ENCAP_UART6:
			break;
		case CAN_ENCAP_UART7:
            break;
		default:
			break;
	}
}

void serialport_hal_disable_tx_isr(serialport_desc port_descriptor)
{
	switch(port_descriptor)
	{
		case UART5:
			UARTIntDisable(UART5_BASE, UART_INT_TX);
			break;
		case UART6:
			UARTIntDisable(UART6_BASE, UART_INT_TX);
			break;
		case UART7:
            UARTIntDisable(UART7_BASE, UART_INT_TX);
			break;
		case CAN_ENCAP_UART5:
            CANMessageClear(CAN0_BASE, 1);
			break;
		case CAN_ENCAP_UART6:
			CANMessageClear(CAN0_BASE, 2);
            break;
		case CAN_ENCAP_UART7:
            CANMessageClear(CAN0_BASE, 3);
			break;
		default:
			break;
	}
}

int serialport_send_byte(serialport_desc port_descriptor, uint8_t byte_to_send)
{
	switch(port_descriptor)
	{
		case UART5:
            UARTCharPut(UART5_BASE, byte_to_send);
			break;
		case UART6:
            UARTCharPut(UART6_BASE, byte_to_send);
			break;
        case UART7:
            UARTCharPut(UART7_BASE, byte_to_send);
            break;
		case CAN_ENCAP_UART5:
            can0_uart_relay(port_descriptor, byte_to_send);
			break;
		case CAN_ENCAP_UART6:
            can0_uart_relay(port_descriptor, byte_to_send);
			break;
		case CAN_ENCAP_UART7:
            can0_uart_relay(port_descriptor, byte_to_send);
			break;
		default:
			return -1;
	}
    return -1; // Return error code if we get here for whatever reason...
}

uint8_t serialport_receive_byte(serialport_desc port_descriptor)
{
    uint8_t can_rx_data[8];
    tCANMsgObject rx;
    
    rx.pui8MsgData = can_rx_data;

	switch(port_descriptor)
	{
		case UART5:
			return (uint8_t)UARTCharGetNonBlocking(UART5_BASE);
		case UART6:
			return (uint8_t)UARTCharGetNonBlocking(UART6_BASE);
		case UART7:
			return (uint8_t)UARTCharGetNonBlocking(UART7_BASE);
		case CAN_ENCAP_UART5:
            CANMessageGet(CAN0_BASE, 4, &rx, 1);
			return can_rx_data[0];
		case CAN_ENCAP_UART6:
            CANMessageGet(CAN0_BASE, 5, &rx, 1);
            return can_rx_data[0];
		case CAN_ENCAP_UART7:
            CANMessageGet(CAN0_BASE, 6, &rx, 1);
            return can_rx_data[0];
		default:
			return 0;
	}
}
