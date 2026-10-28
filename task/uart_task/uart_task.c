//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"
#include "usart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../bsp/uart/uart.h"
#include "../../bus/uart/uart_all.h"
#include"../../application/global_data.h"

// 诊断阶段及最近一次阻塞发送结果，完全无打印时可在调试器中查看。
volatile uint32_t g_uart_diag_stage = 0U;
volatile int32_t g_uart_diag_last_tx_status = -1;

static void uart_diag_send(const char *message)
{
    g_uart_diag_last_tx_status = -2; // 发送进行中
    g_uart_diag_last_tx_status = (int32_t)HAL_UART_Transmit(&huart1,
        (const uint8_t *)message, (uint16_t)strlen(message), 50U);
}

void uart_task(void)
{
    struct uart_device *uart1;
    struct uart_device *uart7;
    struct uart_device *uart10;
    struct uart_device *uart5;
    static char diag_buffer[256];
    uint32_t sequence = 0U;
    uint32_t tx_before = huart1.gState;
    int32_t abort_status = -1;

    g_uart_diag_stage = 1U;
    // 仅在入口解除已有的发送忙状态，不中止接收，不在循环中反复重启。
    if (huart1.gState == HAL_UART_STATE_BUSY_TX) {
        abort_status = (int32_t)HAL_UART_AbortTransmit(&huart1);
    }
    (void)snprintf(diag_buffer, sizeof(diag_buffer),
        "DIAG enter tick=%lu U1_tx_before=0x%lX abort=%ld\r\n",
        (unsigned long)osKernelGetTickCount(), (unsigned long)tx_before, (long)abort_status);
    uart_diag_send(diag_buffer);

    g_uart_diag_stage = 2U;
    uart_diag_send("DIAG before UART1 init\r\n");
    uart1 = uart_get_device("uart1_dma");
    g_uart_diag_stage = 3U;
    uart_diag_send("DIAG before UART7 init\r\n");
    uart7 = uart_get_device("uart7_dma");
    g_uart_diag_stage = 4U;
    uart_diag_send("DIAG before UART10 init\r\n");
    uart10 = uart_get_device("uart10_dma");
    g_uart_diag_stage = 5U;
    uart_diag_send("DIAG before UART5 init\r\n");
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

    g_uart_diag_stage = 6U;
    (void)snprintf(diag_buffer, sizeof(diag_buffer),
        "DIAG init done ports=%u/%u/%u/%u wait=2000\r\n",
        (unsigned int)(uart1 != NULL), (unsigned int)(uart7 != NULL),
        (unsigned int)(uart10 != NULL), (unsigned int)(uart5 != NULL));
    uart_diag_send(diag_buffer);
    osDelay(2000);

    for (;;)
    {
        // int32_t previous_tx = g_uart_diag_last_tx_status;
        //
        // g_uart_diag_stage = 7U;
        // ++sequence;
        // (void)snprintf(diag_buffer, sizeof(diag_buffer),
        //     "DIAG n=%lu tick=%lu prev_tx=%ld U1_tx=0x%lX U1_rx=0x%lX U1_err=0x%lX U1_dma_tx=%lu U1_tx_left=%lu\r\n",
        //     (unsigned long)sequence, (unsigned long)osKernelGetTickCount(), (long)previous_tx,
        //     (unsigned long)huart1.gState, (unsigned long)huart1.RxState, (unsigned long)huart1.ErrorCode,
        //     (huart1.hdmatx != NULL) ? (unsigned long)huart1.hdmatx->State : 0UL,
        //     (huart1.hdmatx != NULL) ? (unsigned long)__HAL_DMA_GET_COUNTER(huart1.hdmatx) : 0UL);
        // uart_diag_send(diag_buffer);
        // osDelay(50U);
        //
        // g_uart_diag_stage = 8U;
        // (void)snprintf(diag_buffer, sizeof(diag_buffer),
        //     "UART ports=%u/%u/%u/%u start=%ld rx=%lu len=%u err=%lu last=0x%lX dma_err=0x%lX state=0x%lX dma=%lu dmar=%u idle=%u ndtr=%lu ready=%u age_tick=%lu ch1=%d ch3=%d s1=%u s2=%u\r\n",
        //     (unsigned int)(uart1 != NULL), (unsigned int)(uart7 != NULL),
        //     (unsigned int)(uart10 != NULL), (unsigned int)(uart5 != NULL),
        //     (long)g_uart5_last_start_status, (unsigned long)g_uart5_rx_event_count,
        //     (unsigned int)g_uart5_last_rx_length, (unsigned long)g_uart5_error_count,
        //     (unsigned long)g_uart5_last_error, (unsigned long)g_uart5_last_dma_error,
        //     (unsigned long)huart5.RxState,
        //     (huart5.hdmarx != NULL) ? (unsigned long)huart5.hdmarx->State : 0UL,
        //     (unsigned int)((huart5.Instance->CR3 & USART_CR3_DMAR) != 0U),
        //     (unsigned int)((huart5.Instance->CR1 & USART_CR1_IDLEIE) != 0U),
        //     (huart5.hdmarx != NULL) ? (unsigned long)__HAL_DMA_GET_COUNTER(huart5.hdmarx) : 0UL,
        //     (unsigned int)global_data.remote_data.rc.remote_ready,
        //     (unsigned long)(osKernelGetTickCount() - global_data.remote_data.rc.remote_update_tick),
        //     global_data.remote_data.rc.ch1, global_data.remote_data.rc.ch3,
        //     (unsigned int)global_data.remote_data.rc.s1, (unsigned int)global_data.remote_data.rc.s2);
        // uart_diag_send(diag_buffer);
        osDelay(450U);



    }
}
