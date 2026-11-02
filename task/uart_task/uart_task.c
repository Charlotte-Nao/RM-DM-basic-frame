//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"

#include <stdint.h>

#include "../../bsp/uart/uart.h"
#include "../../bus/uart/uart_all.h"
#include"../../application/global_data.h"
#include "../../dsp/math.h"
#include "../../device/motor/motor_internal.h"



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
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    dm8009p_data_t *lf_data = lf_motor->motor_data;
    dm8009p_data_t *lb_data = lb_motor->motor_data;
    dm8009p_data_t *rf_data = rf_motor->motor_data;
    dm8009p_data_t *rb_data = rb_motor->motor_data;

    osDelay(2000);
    uint32_t next_wake_tick = osKernelGetTickCount();
    uint32_t period_tick = osKernelGetTickFreq() / 10U;

    for (;;)
    {
        uart1->uart_printf(uart1,
            "TORQUE_Nm LF=%.3f "
            "LB=%.3f "
            "RF=%.3f "
            "RB=%.3f\r\n",
            lf_data->output_torque_nm,
            lb_data->output_torque_nm,
            rf_data->output_torque_nm,
            rb_data->output_torque_nm);

            osDelay(10);



    }
}
