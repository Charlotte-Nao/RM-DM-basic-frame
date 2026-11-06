//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"

#include <stdint.h>

#include "../../bsp/uart/uart.h"
#include "../../bus/uart/uart_all.h"
#include"../../application/global_data.h"
#include "../../device/motor/motor.h"
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

    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");

    osDelay(2000);

    for (;;)
    {
        float lf_raw_pos = 0.0f;
        float lb_raw_pos = 0.0f;
        lf_motor->get_status(lf_motor, "POS", &lf_raw_pos);
        lb_motor->get_status(lb_motor, "POS", &lb_raw_pos);

        uart1->uart_printf(uart1,
            "LF_raw_rad=%.6f LF_raw_deg=%.4f LB_raw_rad=%.6f LB_raw_deg=%.4f\r\n"
            "LEFT phi1_deg=%.4f phi4_deg=%.4f L_mm=%.1f phi0_deg=%.4f\r\n",
            lf_raw_pos, lf_raw_pos * RAD_TO_DEG, lb_raw_pos, lb_raw_pos * RAD_TO_DEG,
            global_data.middle_data.l_phi1 * RAD_TO_DEG,
            global_data.middle_data.l_phi4 * RAD_TO_DEG,
            global_data.middle_data.L_l * 1000.0f,
            global_data.middle_data.l_phi0 * RAD_TO_DEG);

        osDelay(100U);



    }
}
