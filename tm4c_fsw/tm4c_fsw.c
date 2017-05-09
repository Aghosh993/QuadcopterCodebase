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
#include <stdio.h>

#include "hal_common_includes.h"

#include "interrupt_utils.h"

#include "mission_timekeeper.h"
#include "serial_comms_highlevel.h"
#include "sf10_reader.h"
#include "bno055_reader.h"
#include "system_shell.h"

#define SERIAL_BUF_LEN  SERIAL_BUFFER_SIZE
#define HELLO_WORLD_TEST    1

volatile serialport uart5_port, uart6_port, uart7_port;

#if defined USE_CAN_ENCAP
    volatile serialport uart5_can_port, uart6_can_port, uart7_can_port;
#endif

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
        serialport_highlevel_tx_isr(&uart5_port);
    }

    if(ulStatus & UART_INT_RX)
    {
        serialport_highlevel_rx_isr(&uart5_port);
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
        serialport_highlevel_tx_isr(&uart6_port);
    }

    if(ulStatus & UART_INT_RX)
    {
        serialport_highlevel_rx_isr(&uart6_port);
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
        serialport_highlevel_tx_isr(&uart7_port);
    }

    if(ulStatus & UART_INT_RX)
    {
        serialport_highlevel_rx_isr(&uart7_port);
    }
}

#if defined USE_CAN_ENCAP   

    void foo(void)
    {
        while(1);
    }

    volatile uint8_t can_err_flag;
    volatile uint32_t can_err_counter;

    volatile uint32_t can_tx_ok_counter;
    volatile uint32_t can_rx_ok_counter;

    void CANIntHandler(void)
    {
        static volatile tCANMsgObject sMsgObjectRx;
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
            else
            {
                if(ui32Status & CAN_STATUS_RXOK)
                {
                    ++can_rx_ok_counter;
                }
                else
                {
                    foo();
                }            
            }
            // if(ui32Status & (CAN_STATUS_TXOK | CAN_STATUS_RXOK) == 0)
            // {
            //     can_err_flag = 1U;
            //     ++can_err_counter;
            // }
            // if(ui32Status & (CAN_STATUS_BUS_OFF | CAN_STATUS_LEC_STUFF | CAN_STATUS_LEC_ACK))
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
                    serialport_highlevel_tx_isr(&uart7_can_port);
                    // if(serialport_tx_buf_empty(uart5_can_port_ptr))
                    // {
                    //     serialport_highlevel_tx_isr(uart5_can_port_ptr);
                    // }
                    // else
                    // {
                    //     if(serialport_tx_buf_empty(uart6_can_port_ptr))
                    //     {
                    //         serialport_highlevel_tx_isr(uart6_can_port_ptr);
                    //     }
                    //     else
                    //     {
                    //         if(serialport_tx_buf_empty(uart7_can_port_ptr))
                    //         {
                    //             serialport_highlevel_tx_isr(uart7_can_port_ptr);
                    //         }
                    //     }
                    // }
                    break;
                case 2:
                    CANMessageGet(CAN0_BASE, 2, &sMsgObjectRx, true);
                    // serialport_highlevel_tx_isr(uart6_can_port_ptr);
                    break;
                case 3:
                    CANMessageGet(CAN0_BASE, 3, &sMsgObjectRx, true);
                    // serialport_highlevel_tx_isr(uart7_can_port_ptr);
                    break;
                case 4:
                    CANIntClear(CAN0_BASE, 4);
                    serialport_highlevel_rx_isr(&uart5_can_port);
                    break;
                case 5:
                    CANIntClear(CAN0_BASE, 5);
                    serialport_highlevel_rx_isr(&uart6_can_port);
                    break;
                case 6:
                    CANIntClear(CAN0_BASE, 6);
                    serialport_highlevel_rx_isr(&uart7_can_port);
                    break;
                default:
                    can_err_flag = 2U;
                    ++can_err_counter;
                    CANIntClear(CAN0_BASE, ui32Status);
                    CANMessageGet(CAN0_BASE, ui32Status, &sMsgObjectRx, 1);
                    break;
            }
            // CANIntClear(CAN0_BASE, ui32Status);
        }
    }
#endif

#if !defined USE_CAN_ENCAP

    void CANIntHandler(void)
    {
        static volatile tCANMsgObject sMsgObjectRx;
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
            // Do something here for unexpected stuff...    
        }
        else // Process message-related interrupts here:
        {
            switch(ui32Status)
            {
                case 1:
                    CANMessageGet(CAN0_BASE, 1, &sMsgObjectRx, true);
                    break;
                case 4:
                    CANIntClear(CAN0_BASE, 4);
                    break;
                case 5:
                    CANIntClear(CAN0_BASE, 5);
                    break;
                case 6:
                    CANIntClear(CAN0_BASE, 6);
                default:
                    CANIntClear(CAN0_BASE, ui32Status);
                    CANMessageGet(CAN0_BASE, ui32Status, &sMsgObjectRx, 1);
                    break;
            }
        }
    }
#endif

#if !defined USE_CAN_ENCAP
    static void can0_init_noencap(void)
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
        
        CANEnable(CAN0_BASE);

        CANRetrySet(CAN0_BASE, true);
    }
#endif

static volatile sf10_sensor_data_handler sf10_handler;

void Systick_Handler(void)
{
    flag_scheduler_callback();
    update_mission_time_counter();

    /*
        Process SF11/C height sensor data:
     */
    sf10_reader_process_bytes(&sf10_handler);
}

// Function to configure the system tick
static void enable_systick(void)
{
    SysTickIntDisable();
    SysTickDisable();
    SysTickPeriodSet(80000); // For 1 kHz timebase
    SysTickEnable();
    SysTickIntEnable();
}

typedef enum {
    HEIGHT_HEADING_MSG,
    FLOW_MSG
} sensor_msg;

static void can_send_databuf(uint8_t* buf, int len, sensor_msg m_type)
{
    tCANMsgObject sCANMessage;
    sCANMessage.ui32MsgIDMask = 0xF; // Do we need this for TX objects?? We'll find out :)
    sCANMessage.ui32Flags = 0;
    sCANMessage.ui32MsgLen = (uint32_t)len;
    sCANMessage.pui8MsgData = buf;
    switch(m_type)
    {
        case HEIGHT_HEADING_MSG:
            sCANMessage.ui32MsgID = 0x00000001;
            break;
        case FLOW_MSG:
            sCANMessage.ui32MsgID = 0x00000002;
            break;
    }
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);
}

void send_sensor_msg(float* msg_data, sensor_msg m)
{
    union {
        float input[2];
        uint8_t output[8];
    } f32_2_to_uint8;

    f32_2_to_uint8.input[0] = msg_data[0];
    f32_2_to_uint8.input[1] = msg_data[1];

    can_send_databuf(f32_2_to_uint8.output, 8, m);
}

int main(void)
{
    _disable_interrupts();
    /*
        Zero all error flags and CAN bus counters: 
     */
    #if defined USE_CAN_ENCAP
        can_err_counter = 0U;
        can_tx_ok_counter = 0U;
        can_rx_ok_counter = 0U;
        can_err_flag = 0U;
    #endif

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    serialport_hal_init();

    #if !defined USE_CAN_ENCAP
        can0_init_noencap();
    #endif

    #if defined USE_CAN_ENCAP

        serialport_init(&uart5_port, UART5);

        serialport_init(&uart6_port, UART6);

        serialport_init(&uart7_port, UART7);

        serialport_init(&uart5_can_port, CAN_ENCAP_UART5);

        serialport_init(&uart6_can_port, CAN_ENCAP_UART6);

        serialport_init(&uart7_can_port, CAN_ENCAP_UART7);

    #endif

    enable_systick();

    _enable_interrupts();

    #if !defined USE_CAN_ENCAP
        init_new_sf10_data_handler(&sf10_handler, 200U, MAX_HEIGHT_SF11_C, &uart6_port, UART6);
        timekeeper_delay(1000); // Allow the BNO055 sensor to initialize 
        BNO055_init(&uart5_port, UART5);
    #endif

    #if !defined USE_CAN_ENCAP
        uint8_t sf10_25hz_trigger_flag = create_flag(39U);

        float height_sensor_reading = 0.0f;
        int i = 0;
        float height_heading_telem[2];
        height_heading_telem[0] = 0.0f;
        height_heading_telem[1] = 0.0f;
    #endif

    #if defined ENABLE_SYSTEM_SHELL
        setup_system_shell();
        while(1)
        {
            shell_run();
        }
    #endif

    #if defined BNO055_PASSTHRU_TEST

        // BNO055 Passthru test, linking UART5 and UART7:

        char recvbuf[50];
        uint32_t recv_bytes;

        serialport_init(&uart5_port, UART5);
        serialport_init(&uart7_port, UART7);

        printf("Hello!!\r\n");

        while(1)
        {
            recv_bytes = serialport_receive_data_buffer(&uart5_port, recvbuf, 50);
            if(recv_bytes > 0)
            {
                serialport_send_data_buffer(&uart7_port, recvbuf, recv_bytes);
            }

            recv_bytes = serialport_receive_data_buffer(&uart7_port, recvbuf, 50);
            if(recv_bytes > 0)
            {
                serialport_send_data_buffer(&uart5_port, recvbuf, recv_bytes);
            }
        }

    #endif

    #if defined BNO055_BLOCKING_READ_TEST

        // Blocking-mode BNO055 read test:

        char recvbuf[50];
        uint32_t recv_bytes;

        serialport_init(&uart5_port, UART5);
        serialport_init(&uart7_port, UART7);

        uint8_t bno_init1[5] = {0xAA, 0x00, 0x3d, 0x01, 0x00};
        uint8_t bno_init2[5] = {0xAA, 0x00, 0x3d, 0x01, 0x0C};
        uint8_t bno_read_req[4] = {0xAA, 0x01, 0x14, 20U};
        printf("Starting...\r\n");

        serialport_send_data_buffer_blocking(&uart5_port, bno_init1, 5);
        serialport_receive_data_buffer_blocking(&uart5_port, bno_init1, 2);
        serialport_send_data_buffer_blocking(&uart5_port, bno_init2, 5);
        serialport_receive_data_buffer_blocking(&uart5_port, bno_init2, 2);
        union {
            uint8_t input[2];
            int16_t output;
        } conv_to_i16;

        while(1)
        {
            serialport_send_data_buffer_blocking(&uart5_port, bno_read_req, 4);
            serialport_receive_data_buffer_blocking(&uart5_port, recvbuf, 2);
            if(recvbuf[0] == 0xBB)
            {
                recv_bytes = recvbuf[1];
                serialport_receive_data_buffer_blocking(&uart5_port, recvbuf, recv_bytes);
                conv_to_i16.input[0] = recvbuf[6];
                conv_to_i16.input[1] = recvbuf[7];
                printf("Got data %f\r\n", (float)conv_to_i16.output/(float)16);
            }
        }
    #endif

    BNO055_trigger_get_data();

    while(1)
    {
        #if !defined USE_CAN_ENCAP
            /*
                Process BNO055 sensor data:
            */
            BNO055_recv_callback();

            if(get_flag_state(sf10_25hz_trigger_flag) == STATE_PENDING)
            {
                reset_flag(sf10_25hz_trigger_flag);
                send_sensor_msg(height_heading_telem, HEIGHT_HEADING_MSG);
                request_sf10_sensor_update(&sf10_handler);
                BNO055_trigger_get_data();
            }

            if(sf10_received_new_data(&sf10_handler))
            {
                height_heading_telem[0] = get_last_sf10_sensor_height(&sf10_handler);
            }

            if(BNO055_received_new_data())
            {
                height_heading_telem[1] = BNO055_get_heading();
            }
        #endif
    }
}
