//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"

#include <stdint.h>

#include "../../bsp/uart/uart.h"
#include "../../bus/uart/uart_all.h"
#include"../../application/global_data.h"
#include "../../dsp/LegLength/leg_length.h"
#include "../../dsp/math.h"



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

    osDelay(2000);

    for (;;)
    {
        float target_l_mm = target_length_l * 1000.0f;
        float target_r_mm = target_length_r * 1000.0f;
        float length_l_mm = global_data.middle_data.L_l * 1000.0f;
        float length_r_mm = global_data.middle_data.L_r * 1000.0f;

        uart1->uart_printf(uart1,
            "LEFT target_mm=%.1f actual_mm=%.1f error_mm=%.1f \r\n"
            "RIGHT target_mm=%.1f actual_mm=%.1f error_mm=%.1f \r\n",
            target_l_mm, length_l_mm, target_l_mm - length_l_mm,
            target_r_mm, length_r_mm, target_r_mm - length_r_mm);
        osDelay(500U);



    }
}
