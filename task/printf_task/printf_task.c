//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../bsp/uart/uart.h"
#include "../../application/global_data.h"

void printf_task(void)
{
    struct uart_device *uart10 = uart_get_device("uart10_dma");

    if (uart10 == NULL) {
        return;
    }

    for (;;) {
        (void)uart10->uart_printf(uart10,
                                 "host_data.roll: %.4f\r\n",
                                 (double)host_data.roll);
        osDelay(1000U);
    }
}
