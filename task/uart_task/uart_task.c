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
#include "../../dsp/VMC/vmc.h"
#include "../../dsp/math.h"
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


    osDelay(2000);
    for (;;)
    {
        uart1->uart_printf(uart1,
            "r_theta=%.3f     L_r=%.3f\r\n",
            global_data.middle_data.r_theta * RAD_TO_DEG,
            global_data.middle_data.L_r);

        osDelay(100);

        uart1->uart_printf(uart1,
            "l_theta=%.3f     L_l=%.3f\r\n",
            global_data.middle_data.l_theta * RAD_TO_DEG,
            global_data.middle_data.L_l);

        osDelay(1000);



    }
}
