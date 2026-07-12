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

/** A UART instance using either interrupt or DMA transmission. */
struct uart_device {
    const char *name;

    /** Start this device's receive operation. Safe to call more than once. */
    int (*uart_init)(struct uart_device *device);

    /** Send a NUL-terminated string asynchronously. */
    int (*uart_send)(struct uart_device *device, const char *message);

    /** Format and send text asynchronously; completion uses uart_send_callback. */
    int (*uart_printf)(struct uart_device *device, const char *format, ...);

    /** Called in interrupt context after a complete IT/DMA transmission. */
    uart_send_callback_t uart_send_callback;

    /** Called in interrupt context when data is received. */
    uart_recv_callback_t uart_recv_callback;

    void *priv_data;
};

/**
 * Return an initialised UART device by name, or NULL if the name is unknown.
 *
 * Available names: uart1_it, uart1_dma, uart5_it, uart5_dma, uart7_it,
 * uart7_dma, uart10_it, uart10_dma. UART5 is receive-only on this board.
 */
struct uart_device *uart_get_device(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* DM_UART_H */
