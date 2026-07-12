//
// Created by charlotte on 7/12/26.
//

#include "test_task.h"
#include "cmsis_os2.h"
#include "../bsp/LED/LED.h"
#include "../bsp/uart/uart.h"

int test = 0;

static void uart1_send_complete(struct uart_device *device)
{
    (void)device;
    test ++;
}

void test_task(void)
{
    static const uint8_t message[] = "uart_test: %d\r\n",&test;
    struct uart_device *uart1 = uart_get_device("uart1_dma");

    if (uart1 == NULL) {
        LED_RED_SET();
    }

    uart1->uart_send_callback = uart1_send_complete;

    for (;;)
    {
        (void)uart1->uart_send(uart1, message, sizeof(message) - 1U);
        osDelay(1000U);
    }

}
