//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"
#include "usart.h"

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
    if (uart1 != NULL) {
        uart1->uart_recv_callback = uart1_recv_callback;
    }
    if (uart7 != NULL) {
        uart7->uart_recv_callback = uart7_recv_callback;
    }
    if (uart10 != NULL) {
        uart10->uart_recv_callback = uart10_recv_callback;
    }
    if (uart5 != NULL) {
        uart5->uart_recv_callback = uart5_recv_callback;
    }

    osDelay(2000);

    for (;;)
    {
        float target_l_mm = target_length_l * 1000.0f;
        float target_r_mm = target_length_r * 1000.0f;
        float length_l_mm = global_data.middle_data.L_l * 1000.0f;
        float length_r_mm = global_data.middle_data.L_r * 1000.0f;

        if (uart1 != NULL) {
            uart1->uart_printf(uart1,
                "LEFT target_mm=%.1f actual_mm=%.1f error_mm=%.1f \r\n"
                "RIGHT target_mm=%.1f actual_mm=%.1f error_mm=%.1f \r\n",
                target_l_mm, length_l_mm, target_l_mm - length_l_mm,
                target_r_mm, length_r_mm, target_r_mm - length_r_mm);
        }
        osDelay(50U);

        if (uart1 != NULL) {
            uart1->uart_printf(uart1,
                "UART ports=%u/%u/%u/%u start=%ld rx=%lu len=%u err=%lu last=0x%lX dma_err=0x%lX state=0x%lX dma=%lu dmar=%u idle=%u ndtr=%lu ready=%u age_tick=%lu ch1=%d ch3=%d s1=%u s2=%u\r\n",
                (unsigned int)(uart1 != NULL), (unsigned int)(uart7 != NULL),
                (unsigned int)(uart10 != NULL), (unsigned int)(uart5 != NULL),
                (long)g_uart5_last_start_status, (unsigned long)g_uart5_rx_event_count,
                (unsigned int)g_uart5_last_rx_length, (unsigned long)g_uart5_error_count,
                (unsigned long)g_uart5_last_error, (unsigned long)g_uart5_last_dma_error,
                (unsigned long)huart5.RxState, (unsigned long)huart5.hdmarx->State,
                (unsigned int)((huart5.Instance->CR3 & USART_CR3_DMAR) != 0U),
                (unsigned int)((huart5.Instance->CR1 & USART_CR1_IDLEIE) != 0U),
                (unsigned long)__HAL_DMA_GET_COUNTER(huart5.hdmarx),
                (unsigned int)global_data.remote_data.rc.remote_ready,
                (unsigned long)(osKernelGetTickCount() - global_data.remote_data.rc.remote_update_tick),
                global_data.remote_data.rc.ch1, global_data.remote_data.rc.ch3,
                (unsigned int)global_data.remote_data.rc.s1, (unsigned int)global_data.remote_data.rc.s2);
        }
        osDelay(450U);



    }
}
