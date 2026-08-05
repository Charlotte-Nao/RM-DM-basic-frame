//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../bsp/uart/uart.h"
#include "../../device/motor/motor.h"

#define PRINTF_TASK_PERIOD_MS 20U

void printf_task(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    struct uart_device *uart7 = uart_get_device("uart7_dma");
    struct uart_device *uart10 = uart_get_device("uart10_dma");
    uint32_t next_wake_tick = osKernelGetTickCount();

    if (uart1 == NULL || uart7 == NULL || uart10 == NULL) {
        return;
    }

    for (;;) {



        next_wake_tick += PRINTF_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);
    }
}
