//
// Created by charlotte on 7/12/26.
//

#include "test_task.h"
#include "cmsis_os2.h"
#include "../bsp/uart/uart.h"

#include <stdbool.h>
#include <string.h>

#define UART_TEST_RX_STRING_SIZE 256U

static char uart1_rx_string[UART_TEST_RX_STRING_SIZE];
static volatile bool uart1_rx_ready;

/* Invoked from the UART DMA receive interrupt context. */
static void uart1_receive_complete(struct uart_device *device,
                                   const uint8_t *data,
                                   uint16_t length)
{
    uint16_t copy_length;

    (void)device;
    if (uart1_rx_ready) {
        return;
    }

    copy_length = (length < (UART_TEST_RX_STRING_SIZE - 1U))
                    ? length : (UART_TEST_RX_STRING_SIZE - 1U);
    memcpy(uart1_rx_string, data, copy_length);
    uart1_rx_string[copy_length] = '\0';
    uart1_rx_ready = true;
}

void test_task(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");

    if (uart1 == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    uart1->uart_recv_callback = uart1_receive_complete;

    for (;;)
    {
        if (uart1_rx_ready) {
            /* uart_send copies this string into its own DMA-safe TX buffer. */
            if (uart1->uart_send(uart1, uart1_rx_string) == 0) {
                uart1_rx_ready = false;
            }
        }
        osDelay(1U);
    }

}
