//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"

#include <stdint.h>

#include "../../bsp/uart/uart.h"
#include "../../protocol/protocol.h"
#include"../../application/global_data.h"
#include "../../bsp/LED/LED.h"

static void uart1_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
}
static void uart7_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
}
static void uart10_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
}

void uart_task(void)
{
    struct uart_device *uart1;
    struct uart_device *uart7;
    struct uart_device *uart10;
    uart1 = uart_get_device("uart1_dma");
    uart7 = uart_get_device("uart7_dma");
    uart10 = uart_get_device("uart10_dma");
    uart1->uart_recv_callback = uart1_recv_callback;
    uart7->uart_recv_callback = uart7_recv_callback;
    uart10->uart_recv_callback = uart10_recv_callback;

    for (;;)
    {
        osDelay(1U);
    }
}
