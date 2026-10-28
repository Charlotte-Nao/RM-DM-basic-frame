/**
 * @file uart.h
 * @brief UART device abstraction for the DM-MC-Board02.
 */

#ifndef DM_UART_H
#define DM_UART_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uart_device;

typedef void (*uart_send_callback_t)(struct uart_device *device);
typedef void (*uart_recv_callback_t)(struct uart_device *device,
                                     const uint8_t *data,
                                     uint16_t length);

struct uart_device {
    const char *name;
    int (*uart_init)(struct uart_device *device);
    int (*uart_send_bytes)(struct uart_device *device,const uint8_t *data,uint16_t length);
    int (*uart_send)(struct uart_device *device, const char *message);
    int (*uart_printf)(struct uart_device *device, const char *format, ...);
    uart_send_callback_t uart_send_callback;
    uart_recv_callback_t uart_recv_callback;
    void *priv_data;
};

struct uart_device *uart_get_device(const char *name);

/* UART5 接收诊断；start_status 为 -1 表示未启动，否则为 HAL 返回值。 */
extern volatile uint32_t g_uart5_rx_event_count;
extern volatile uint16_t g_uart5_last_rx_length;
extern volatile uint32_t g_uart5_error_count;
extern volatile uint32_t g_uart5_last_error;
extern volatile uint32_t g_uart5_last_dma_error;
extern volatile int32_t g_uart5_last_start_status;

#ifdef __cplusplus
}
#endif

#endif /* DM_UART_H */
