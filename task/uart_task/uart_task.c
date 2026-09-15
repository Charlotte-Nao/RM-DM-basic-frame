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

        uart1->uart_printf(uart1,"%d\r\n ",global_data.remote_data.rc.remote_update_tick);
        osDelay(100U);
        uart1->uart_printf(uart1,"%d\r\n ",global_data.remote_data.rc.remote_ready);
        osDelay(100U);
        uart1->uart_printf(uart1,"%d \r\n",global_data.remote_data.rc.ch0);
        osDelay(100U);
        uart1->uart_printf(uart1,"111 \r\n");
        osDelay(100U);

    }
}
