//*****************************************************************************
//
//
// Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.2.111 of the Tiva Firmware Development Package.
//
//*****************************************************************************

/*
    This program performs multiplexer-demultiplexer functions and muxes data from multiple UART ports onto a CAN bus
    for de-multiplexing by another processor.

    (c) 2017, Abhimanyu Ghosh
 */

#include <stdbool.h>
#include <stdint.h>

#include "hal_common_includes.h"
#include "interrupt_utils.h"
#include "serial_comms_highlevel.h"

#define SERIAL_BUF_LEN  SERIAL_BUFFER_SIZE
#define HELLO_WORLD_TEST    1

volatile serialport uart5_port, uart6_port, uart7_port;
volatile serialport uart5_can_port, uart6_can_port, uart7_can_port;

volatile serialport *uart5_port_ptr;
volatile serialport *uart6_port_ptr;
volatile serialport *uart7_port_ptr;

volatile serialport *uart5_can_port_ptr;
volatile serialport *uart6_can_port_ptr;
volatile serialport *uart7_can_port_ptr;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void UART5IntHandler(void){
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART5_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART5_BASE, ulStatus);

    if(ulStatus & UART_INT_TX)
    {
        serialport_highlevel_tx_isr(uart5_port_ptr);
    }

    if(ulStatus & UART_INT_RX)
    {
        serialport_highlevel_rx_isr(uart5_port_ptr);
    }
}

void UART6IntHandler(void){
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART6_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART6_BASE, ulStatus);

    if(ulStatus & UART_INT_TX)
    {
        serialport_highlevel_tx_isr(uart6_port_ptr);
    }

    if(ulStatus & UART_INT_RX)
    {
        serialport_highlevel_rx_isr(uart6_port_ptr);
    }
}

void UART7IntHandler(void){
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART7_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART7_BASE, ulStatus);

    if(ulStatus & UART_INT_TX)
    {
        serialport_highlevel_tx_isr(uart7_port_ptr);
    }

    if(ulStatus & UART_INT_RX)
    {
        serialport_highlevel_rx_isr(uart7_port_ptr);
        // uint8_t c = (uint8_t)UARTCharGetNonBlocking(UART7_BASE);
        // UARTCharPut(UART7_BASE, c);
    }
}

volatile uint8_t can_err_flag;
volatile uint32_t can_err_counter;

volatile uint32_t can_tx_ok_counter;
volatile uint32_t can_rx_ok_counter;

void CANIntHandler(void)
{
    tCANMsgObject sMsgObjectRx;
    uint32_t ui32Status;
    uint8_t msg_data[8];
    sMsgObjectRx.pui8MsgData = msg_data;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    CANIntClear(CAN0_BASE, ui32Status);
    
    // If the cause is a controller status interrupt, then get the status
    
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        if(ui32Status & CAN_STATUS_TXOK)
        {
            ++can_tx_ok_counter;
        }
        else if(ui32Status == CAN_STATUS_RXOK)
        {
            ++can_rx_ok_counter;
        }
        else
        {
            can_err_flag = 1U;
            ++can_err_counter;
        }
    }
    else
    {
        switch(ui32Status)
        {
            /*
                TX interrupts not processed within ISR for now... need to fix this to improve CAN<->UART bandwidth!!!
             */
            case 1:
                CANMessageGet(CAN0_BASE, 1, &sMsgObjectRx, true); // If we ever get here, clear the interrupt through a dummy read.
                break;
            case 2:
                CANMessageGet(CAN0_BASE, 2, &sMsgObjectRx, true);
                break;
            case 3:
                CANMessageGet(CAN0_BASE, 3, &sMsgObjectRx, true);
                break;
            case 4:
                CANIntClear(CAN0_BASE, 4);
                serialport_highlevel_rx_isr(uart5_can_port_ptr);
                break;
            case 5:
                CANIntClear(CAN0_BASE, 5);
                serialport_highlevel_rx_isr(uart6_can_port_ptr);
                break;
            case 6:
                CANIntClear(CAN0_BASE, 6);
                serialport_highlevel_rx_isr(uart7_can_port_ptr);
                break;
            default:
                can_err_flag = 2U;
                ++can_err_counter;
                CANIntClear(CAN0_BASE, ui32Status);
                CANMessageGet(CAN0_BASE, ui32Status, &sMsgObjectRx, 1);
                break;
        }
        CANIntClear(CAN0_BASE, ui32Status);
    }
}

int main(void)
{
    _disable_interrupts();
    /*
        Zero all error flags and CAN bus counters: 
     */
    can_err_counter = 0U;
    can_tx_ok_counter = 0U;
    can_rx_ok_counter = 0U;
    can_err_flag = 0U;

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    serialport_hal_init();

    // _enable_interrupts();
    // while(1);

    // serialport_init(&uart5_port, UART5);
    // uart5_port_ptr = &uart5_port;

    // serialport_init(&uart6_port, UART6);
    // uart6_port_ptr = &uart6_port;

    serialport_init(&uart7_port, UART7);
    uart7_port_ptr = &uart7_port;

    // serialport_init(&uart5_can_port, CAN_ENCAP_UART5);
    // uart5_can_port_ptr = &uart5_can_port;

    // serialport_init(&uart6_can_port, CAN_ENCAP_UART6);
    // uart6_can_port_ptr = &uart6_can_port;

    // serialport_init(&uart7_can_port, CAN_ENCAP_UART7);
    // uart7_can_port_ptr = &uart7_can_port;

    _enable_interrupts();

    #ifdef HELLO_WORLD_TEST
        // serialport_send_data_buffer(uart5_port_ptr, (uint8_t *)"Hello UART5!!\r\n", 15U);
        // serialport_send_data_buffer(uart6_port_ptr, (uint8_t *)"Hello UART6!!\r\n", 15U);
        serialport_send_data_buffer(uart7_port_ptr, (uint8_t *)"Hello UART7!!\r\n", 15U);
        // while(1);

        // serialport_send_data_buffer(uart5_can_port_ptr, (uint8_t *)"Hello CAN_UART5!!\r\n", 19U);
        // serialport_send_data_buffer(uart6_can_port_ptr, (uint8_t *)"Hello CAN_UART6!!\r\n", 19U);
        // serialport_send_data_buffer(uart7_can_port_ptr, (uint8_t *)"Hello CAN_UART7!!\r\n", 19U);
    #endif

    uint8_t buf[SERIAL_BUF_LEN];
    uint32_t bytes_recv;

    uint8_t can_channel = 1U;

    while(1)
    {
        // if(can_err_flag > 0U)
        // {
        //     _disable_interrupts();
        //     while(1)
        //     {
        //         UARTCharPut(UART7_BASE, 'e');
        //     }
        // }

        /*
            Process pending CAN transmissions in a round-robbin fashion.
            If a given "channel" has no bytes in buffer, it'll just return immediately and yield to this "executive"...
         */

        // if(CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST)==0) // If no TX messages are pending, go ahead and transmit whatever is on the buffers
        // {
        //     switch(can_channel)
        //     {
        //         case 1U:
        //             serialport_highlevel_tx_isr(uart5_can_port_ptr);
        //             ++can_channel;
        //             break;
        //         case 2U:
        //             serialport_highlevel_tx_isr(uart6_can_port_ptr);
        //             ++can_channel;
        //             break;
        //         case 3U:
        //             serialport_highlevel_tx_isr(uart7_can_port_ptr);
        //             can_channel = 1U;
        //             break;
        //     }
        // }
        /*
            Forwarding relationships from actual UARTs on TM4C to CAN bus:
            In all cases below, we attempt to read SERIAL_BUF_LEN number of bytes into "buf", in an asynchronous fashion.
         */
        // bytes_recv = serialport_receive_data_buffer(uart5_port_ptr, buf, SERIAL_BUF_LEN);
        // if(bytes_recv > 0U)
        // {
        //     serialport_send_data_buffer(uart5_can_port_ptr, buf, bytes_recv);
        // }

        // bytes_recv = serialport_receive_data_buffer(uart6_port_ptr, buf, SERIAL_BUF_LEN);
        // if(bytes_recv > 0U)
        // {
        //     serialport_send_data_buffer(uart6_can_port_ptr, buf, bytes_recv);
        // }

        bytes_recv = serialport_receive_data_buffer(uart7_port_ptr, buf, SERIAL_BUF_LEN);
        if(bytes_recv > 0U)
        {
            serialport_send_data_buffer(uart7_port_ptr, buf, bytes_recv);
        }

        /*
            Forwarding relationships from CAN bus to actual UARTs on TM4C:
            In all cases below, we attempt to read SERIAL_BUF_LEN number of bytes into "buf", in an asynchronous fashion.
         */
        // bytes_recv = serialport_receive_data_buffer(uart5_can_port_ptr, buf, SERIAL_BUF_LEN);
        // if(bytes_recv > 0U)
        // {
        //     serialport_send_data_buffer(uart5_port_ptr, buf, bytes_recv);
        // }

        // bytes_recv = serialport_receive_data_buffer(uart6_can_port_ptr, buf, SERIAL_BUF_LEN);
        // if(bytes_recv > 0U)
        // {
        //     serialport_send_data_buffer(uart6_port_ptr, buf, bytes_recv);
        // }

        // bytes_recv = serialport_receive_data_buffer(uart7_can_port_ptr, buf, SERIAL_BUF_LEN);
        // if(bytes_recv > 0U)
        // {
        //     serialport_send_data_buffer(uart7_port_ptr, buf, bytes_recv);
        // }
    }
}

// int main(void)
// {
//     _disable_interrupts();
    
//     SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
//     HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock GPIO_CR register with this magic value
//     HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0xFF;

//     GPIOPinConfigure(GPIO_PF0_CAN0RX);
//     GPIOPinConfigure(GPIO_PF3_CAN0TX); 
//     GPIOPinTypeCAN(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_3);

//     SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
//     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));
//     CANInit(CAN0_BASE);
//     CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
//     CANMessageClear(CAN0_BASE, 1);
//     CANMessageClear(CAN0_BASE, 2);
//     CANMessageClear(CAN0_BASE, 3);
//     CANEnable(CAN0_BASE);
//     CANRetrySet(CAN0_BASE, false);

//     tCANMsgObject sCANMessage;
//     uint8_t can_msg_contents[8];
//     can_msg_contents[0] = 0U;

//     sCANMessage.ui32MsgIDMask = 0xF; // Do we need this for TX objects?? We'll find out :)
//     sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
//     sCANMessage.ui32MsgLen = 1;
//     sCANMessage.pui8MsgData = can_msg_contents;
//     sCANMessage.ui32MsgID = 0x60000003;

//     // while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))&0x01!=0);//&0x01==0x01);
//     // while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))&0x02!=0);//&0x01==0x01);
//     // while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))&0x04!=0);//&0x01==0x01);
//     while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))!=0);
//     CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_TX);
//     // uint32_t stat = HWREG(CAN0_BASE + CAN_O_STS);
//     // while(stat & CAN_STS_TXOK == 0)
//     // {
//     //     stat = HWREG(CAN0_BASE + CAN_O_STS);
//     // }
//     // HWREG(CAN0_BASE + CAN_O_STS) = ~(CAN_STS_RXOK | CAN_STS_TXOK |
//     //                                         CAN_STS_LEC_M);

//     while(1)
//     {
//         ++can_msg_contents[0];

//         // while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))&0x01!=0);//&0x01==0x01);
//         // while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))&0x02!=0);//&0x01==0x01);
//         // while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))&0x04!=0);//&0x01==0x01);
//         while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST))!=0);        
//         CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_TX);
//         SysCtlDelay(10000);
//         // while ((CANStatusGet(CAN0_BASE, CAN_STS_CONTROL)&CAN_STS_TXOK)==0);
//         // stat = HWREG(CAN0_BASE + CAN_O_STS);
//         // while(stat & CAN_STS_TXOK == 0)
//         // {
//         //     stat = HWREG(CAN0_BASE + CAN_O_STS);           
//         // }
//         // HWREG(CAN0_BASE + CAN_O_STS) = ~(CAN_STS_RXOK | CAN_STS_TXOK |
//         //                                     CAN_STS_LEC_M);  
//     }
// }