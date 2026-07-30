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

static void uart1_recv_callback(struct uart_device *device,
                                const uint8_t *data,
                                uint16_t length)
{
    struct protocol_data received_data;

    (void)device;

    if (data == NULL) {
        return;
    }

    if (protocol_parse(data, length, &received_data) > 0) {
        host_data.x = received_data.x;
        host_data.y = received_data.y;
        host_data.z = received_data.z;
        host_data.roll = (float)received_data.roll / 10.0f;
        host_data.action = received_data.action;
    }
    LED_PURPLE_SET();
}

void uart_task(void)
{
    struct uart_device *uart1;
    uart1 = uart_get_device("uart1_dma");
    while (uart1 == NULL) {
        osDelay(1000U);
        uart1 = uart_get_device("uart1_dma");
    }

    uart1->uart_recv_callback = uart1_recv_callback;

    for (;;)
    {
        osDelay(1U);
    }
}
