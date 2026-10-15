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
#include "../../dsp/LegLength/leg_length.h"
#include "../../device/motor/motor_internal.h"
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

    dm8009p_data_t *lf_data = motor_get_device("DM8009P_LF")->motor_data;
    dm8009p_data_t *lb_data = motor_get_device("DM_8009P_LB")->motor_data;
    dm8009p_data_t *rf_data = motor_get_device("DM_8009P_RF")->motor_data;
    dm8009p_data_t *rb_data = motor_get_device("DM_8009P_RB")->motor_data;


    osDelay(2000);
    for (;;)
    {
        uart1->uart_printf(uart1,
            "target_L_l=%.3f     L_l=%.3f     target_L_r=%.3f     L_r=%.3f     tao_lf=%.3f     tao_lb=%.3f     tao_rf=%.3f     tao_rb=%.3f\r\n",
            target_length_l,
            global_data.middle_data.L_l,
            target_length_r,
            global_data.middle_data.L_r,
            lf_data->output_torque_nm,
            lb_data->output_torque_nm,
            rf_data->output_torque_nm,
            rb_data->output_torque_nm);

        osDelay(2000);



    }
}
