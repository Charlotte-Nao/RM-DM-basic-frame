/**
 * @file uart.c
 * @brief IT/DMA UART device instances and HAL callback dispatching.
 */

#include "uart.h"

#include <stdbool.h>
#include <string.h>

#include "usart.h"

#define UART_DMA_RX_BUFFER_SIZE 256U
#define UART_DMA_TX_BUFFER_SIZE 256U

#define UART_DEVICE_OK              0
#define UART_DEVICE_ERR_PARAM       (-1)
#define UART_DEVICE_ERR_BUSY        (-2)
#define UART_DEVICE_ERR_UNSUPPORTED (-3)
#define UART_DEVICE_ERR_HAL         (-4)

enum uart_transfer_mode {
    UART_TRANSFER_IT,
    UART_TRANSFER_DMA,
};

struct uart_port_data {
    UART_HandleTypeDef *handle;
    struct uart_device *rx_owner;
    struct uart_device *tx_owner;
    uint8_t it_rx_byte;
    uint8_t *dma_rx_buffer;
    uint8_t *dma_tx_buffer;
    uint16_t dma_last_position;
};

struct uart_device_data {
    struct uart_port_data *port;
    enum uart_transfer_mode transfer_mode;
    bool tx_supported;
    bool initialised;
};

static int uart_device_init(struct uart_device *device);
static int uart_device_send(struct uart_device *device,
                            const uint8_t *data,
                            uint16_t length);

#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

/* DMA1 cannot access DTCM on STM32H723; these buffers are linked into RAM_D2. */
static uint8_t uart1_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart5_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart7_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart10_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart1_dma_tx_buffer[UART_DMA_TX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart7_dma_tx_buffer[UART_DMA_TX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart10_dma_tx_buffer[UART_DMA_TX_BUFFER_SIZE] DMA_BUFFER;

static struct uart_port_data uart1_port = {
    .handle = &huart1, .dma_rx_buffer = uart1_dma_rx_buffer,
    .dma_tx_buffer = uart1_dma_tx_buffer,
};
static struct uart_port_data uart5_port = {
    .handle = &huart5, .dma_rx_buffer = uart5_dma_rx_buffer,
};
static struct uart_port_data uart7_port = {
    .handle = &huart7, .dma_rx_buffer = uart7_dma_rx_buffer,
    .dma_tx_buffer = uart7_dma_tx_buffer,
};
static struct uart_port_data uart10_port = {
    .handle = &huart10, .dma_rx_buffer = uart10_dma_rx_buffer,
    .dma_tx_buffer = uart10_dma_tx_buffer,
};

#define UART_DEVICE_DATA(port_data, mode, can_transmit) \
    { .port = &(port_data), .transfer_mode = (mode), .tx_supported = (can_transmit) }

static struct uart_device_data uart1_it_data =
    UART_DEVICE_DATA(uart1_port, UART_TRANSFER_IT, true);
static struct uart_device_data uart1_dma_data =
    UART_DEVICE_DATA(uart1_port, UART_TRANSFER_DMA, true);
static struct uart_device_data uart5_it_data =
    UART_DEVICE_DATA(uart5_port, UART_TRANSFER_IT, false);
static struct uart_device_data uart5_dma_data =
    UART_DEVICE_DATA(uart5_port, UART_TRANSFER_DMA, false);
static struct uart_device_data uart7_it_data =
    UART_DEVICE_DATA(uart7_port, UART_TRANSFER_IT, true);
static struct uart_device_data uart7_dma_data =
    UART_DEVICE_DATA(uart7_port, UART_TRANSFER_DMA, true);
static struct uart_device_data uart10_it_data =
    UART_DEVICE_DATA(uart10_port, UART_TRANSFER_IT, true);
static struct uart_device_data uart10_dma_data =
    UART_DEVICE_DATA(uart10_port, UART_TRANSFER_DMA, true);

#define UART_DEVICE(name_string, data) \
    { .name = (name_string), .uart_init = uart_device_init, \
      .uart_send = uart_device_send, .priv_data = &(data) }

static struct uart_device uart1_it = UART_DEVICE("uart1_it", uart1_it_data);
static struct uart_device uart1_dma = UART_DEVICE("uart1_dma", uart1_dma_data);
static struct uart_device uart5_it = UART_DEVICE("uart5_it", uart5_it_data);
static struct uart_device uart5_dma = UART_DEVICE("uart5_dma", uart5_dma_data);
static struct uart_device uart7_it = UART_DEVICE("uart7_it", uart7_it_data);
static struct uart_device uart7_dma = UART_DEVICE("uart7_dma", uart7_dma_data);
static struct uart_device uart10_it = UART_DEVICE("uart10_it", uart10_it_data);
static struct uart_device uart10_dma = UART_DEVICE("uart10_dma", uart10_dma_data);

static struct uart_device *const uart_devices[] = {
    &uart1_it, &uart1_dma, &uart5_it, &uart5_dma,
    &uart7_it, &uart7_dma, &uart10_it, &uart10_dma,
};

static struct uart_port_data *uart_port_from_handle(UART_HandleTypeDef *handle)
{
    if (handle == &huart1) {
        return &uart1_port;
    }
    if (handle == &huart5) {
        return &uart5_port;
    }
    if (handle == &huart7) {
        return &uart7_port;
    }
    if (handle == &huart10) {
        return &uart10_port;
    }
    return NULL;
}

static int uart_start_receive(struct uart_device *device)
{
    struct uart_device_data *device_data = device->priv_data;
    struct uart_port_data *port = device_data->port;
    HAL_StatusTypeDef status;

    if (port->rx_owner != NULL && port->rx_owner != device) {
        (void)HAL_UART_AbortReceive(port->handle);
    }

    port->rx_owner = device;
    if (device_data->transfer_mode == UART_TRANSFER_IT) {
        status = HAL_UART_Receive_IT(port->handle, &port->it_rx_byte, 1U);
    } else {
        port->dma_last_position = 0U;
        status = HAL_UARTEx_ReceiveToIdle_DMA(port->handle,
                                               port->dma_rx_buffer,
                                               UART_DMA_RX_BUFFER_SIZE);
        if (status == HAL_OK) {
            __HAL_DMA_DISABLE_IT(port->handle->hdmarx, DMA_IT_HT);
        }
    }

    return (status == HAL_OK) ? UART_DEVICE_OK : UART_DEVICE_ERR_HAL;
}

static int uart_device_init(struct uart_device *device)
{
    struct uart_device_data *device_data;

    if (device == NULL || device->priv_data == NULL) {
        return UART_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;
    if (device_data->initialised && device_data->port->rx_owner == device) {
        return UART_DEVICE_OK;
    }

    if (uart_start_receive(device) != UART_DEVICE_OK) {
        return UART_DEVICE_ERR_HAL;
    }

    device_data->initialised = true;
    return UART_DEVICE_OK;
}

static int uart_device_send(struct uart_device *device,
                            const uint8_t *data,
                            uint16_t length)
{
    struct uart_device_data *device_data;
    struct uart_port_data *port;
    HAL_StatusTypeDef status;

    if (device == NULL || data == NULL || length == 0U || device->priv_data == NULL) {
        return UART_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;
    if (!device_data->tx_supported) {
        return UART_DEVICE_ERR_UNSUPPORTED;
    }

    if (!device_data->initialised && uart_device_init(device) != UART_DEVICE_OK) {
        return UART_DEVICE_ERR_HAL;
    }

    port = device_data->port;
    if (port->tx_owner != NULL || port->handle->gState != HAL_UART_STATE_READY) {
        return UART_DEVICE_ERR_BUSY;
    }

    port->tx_owner = device;
    if (device_data->transfer_mode == UART_TRANSFER_IT) {
        status = HAL_UART_Transmit_IT(port->handle, (uint8_t *)data, length);
    } else {
        if (length > UART_DMA_TX_BUFFER_SIZE) {
            port->tx_owner = NULL;
            return UART_DEVICE_ERR_PARAM;
        }
        memcpy(port->dma_tx_buffer, data, length);
        status = HAL_UART_Transmit_DMA(port->handle, port->dma_tx_buffer, length);
    }

    if (status != HAL_OK) {
        port->tx_owner = NULL;
        return (status == HAL_BUSY) ? UART_DEVICE_ERR_BUSY : UART_DEVICE_ERR_HAL;
    }

    return UART_DEVICE_OK;
}

static void uart_dma_deliver(struct uart_port_data *port, uint16_t position)
{
    struct uart_device *device = port->rx_owner;
    uint16_t previous = port->dma_last_position;

    if (device == NULL || device->uart_recv_callback == NULL) {
        port->dma_last_position = (position == UART_DMA_RX_BUFFER_SIZE) ? 0U : position;
        return;
    }

    if (position > previous) {
        device->uart_recv_callback(device, &port->dma_rx_buffer[previous], position - previous);
    } else if (position < previous) {
        device->uart_recv_callback(device, &port->dma_rx_buffer[previous],
                                   UART_DMA_RX_BUFFER_SIZE - previous);
        if (position > 0U) {
            device->uart_recv_callback(device, port->dma_rx_buffer, position);
        }
    }

    port->dma_last_position = (position == UART_DMA_RX_BUFFER_SIZE) ? 0U : position;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    struct uart_port_data *port = uart_port_from_handle(handle);
    struct uart_device *device;

    if (port == NULL) {
        return;
    }

    device = port->tx_owner;
    port->tx_owner = NULL;
    if (device != NULL && device->uart_send_callback != NULL) {
        device->uart_send_callback(device);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    struct uart_port_data *port = uart_port_from_handle(handle);
    struct uart_device *device;
    struct uart_device_data *device_data;

    if (port == NULL || port->rx_owner == NULL) {
        return;
    }

    device = port->rx_owner;
    device_data = device->priv_data;
    if (device_data->transfer_mode != UART_TRANSFER_IT) {
        return;
    }

    if (device->uart_recv_callback != NULL) {
        device->uart_recv_callback(device, &port->it_rx_byte, 1U);
    }
    (void)HAL_UART_Receive_IT(handle, &port->it_rx_byte, 1U);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t position)
{
    struct uart_port_data *port = uart_port_from_handle(handle);
    struct uart_device_data *device_data;

    if (port == NULL || port->rx_owner == NULL) {
        return;
    }

    device_data = port->rx_owner->priv_data;
    if (device_data->transfer_mode == UART_TRANSFER_DMA) {
        uart_dma_deliver(port, position);
    }
}

struct uart_device *uart_get_device(const char *name)
{
    size_t index;

    if (name == NULL) {
        return NULL;
    }

    for (index = 0U; index < sizeof(uart_devices) / sizeof(uart_devices[0]); ++index) {
        if (strcmp(name, uart_devices[index]->name) == 0) {
            if (uart_device_init(uart_devices[index]) != UART_DEVICE_OK) {
                return NULL;
            }
            return uart_devices[index];
        }
    }
    return NULL;
}
