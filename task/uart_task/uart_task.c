//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"

#include <stdint.h>

#include "../../bsp/uart/uart.h"
#include "../../bus/uart/uart_all.h"
#include "../../protocol/protocol.h"
#include"../../application/global_data.h"
#include "../../bsp/LED/LED.h"



void uart_task(void)
{
    struct uart_device *uart1;
    struct uart_device *uart7;
    struct uart_device *uart10;
    struct uart_device *uart5;
    uart1 = uart_get_device("uart1_dma");
    uart7 = uart_get_device("uart7_dma");
    uart10 = uart_get_device("uart10_dma");
    uart5 = uart_get_device("uart5_dma");
    uart1->uart_recv_callback = uart1_recv_callback;
    uart7->uart_recv_callback = uart7_recv_callback;
    uart10->uart_recv_callback = uart10_recv_callback;
    uart5->uart_recv_callback = uart5_recv_callback;


    for (;;)
    {
        uart1->uart_printf(uart1,
            "ch0=%d ch1=%d ch2=%d ch3=%d s1=%d s2=%d reserved=%d ready=%d tick=%lu\r\n",
            global_data.remote_data.rc.ch0,
            global_data.remote_data.rc.ch1,
            global_data.remote_data.rc.ch2,
            global_data.remote_data.rc.ch3,
            global_data.remote_data.rc.s1,
            global_data.remote_data.rc.s2,
            global_data.remote_data.rc.reserved,
            global_data.remote_data.rc.remote_ready,
            (unsigned long)global_data.remote_data.rc.remote_update_tick);
        osDelay(100U);
    }
}
