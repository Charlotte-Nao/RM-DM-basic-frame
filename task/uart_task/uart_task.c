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
            "l_phi1=%.3f     l_phi4=%.3f     l_beta=%.3f     l_phi0=%.3f     L_l=%.3f     lf_pos=%.3f     lb_pos=%.3f\r\n",
            global_data.middle_data.l_phi1 * RAD_TO_DEG,
            global_data.middle_data.l_phi4 * RAD_TO_DEG,
            l_beta * RAD_TO_DEG,
            global_data.middle_data.l_phi0 * RAD_TO_DEG,
            global_data.middle_data.L_l,
            lf_pos * RAD_TO_DEG,
            lb_pos * RAD_TO_DEG);

        osDelay(500);



    }
}
