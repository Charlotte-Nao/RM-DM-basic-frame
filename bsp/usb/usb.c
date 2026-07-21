//
// Created by charlotte on 7/21/26.
//

/**
 * @file usb.c
 * @brief USB CDC BSP device and receive ring buffer.
 */

#include "usb.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

#define USB_BSP_RX_BUFFER_SIZE      2048U
#define USB_BSP_TX_BUFFER_SIZE      2048U
#define USB_BSP_PRINTF_BUFFER_SIZE   512U

#if (USB_BSP_RX_BUFFER_SIZE == 0U) || \
    ((USB_BSP_RX_BUFFER_SIZE & (USB_BSP_RX_BUFFER_SIZE - 1U)) != 0U)
#error "USB_BSP_RX_BUFFER_SIZE must be a power of two"
#endif

struct usb_device_data {
    uint8_t rx_buffer[USB_BSP_RX_BUFFER_SIZE];
    uint8_t tx_buffer[USB_BSP_TX_BUFFER_SIZE];
    uint8_t printf_buffer[USB_BSP_PRINTF_BUFFER_SIZE];

    volatile uint32_t rx_head;
    volatile uint32_t rx_tail;

    volatile uint32_t rx_byte_count;
    volatile uint32_t tx_byte_count;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t tx_busy_count;
    volatile uint32_t tx_error_count;

    bool initialised;
};

static int usb_device_init(struct usb_device *device);
static int usb_device_send_bytes(struct usb_device *device,
                                 const uint8_t *data,
                                 uint16_t length);
static int usb_device_send(struct usb_device *device, const char *message);
static int usb_device_printf(struct usb_device *device,
                             const char *format,
                             ...);
static uint16_t usb_device_available(struct usb_device *device);
static int usb_device_read(struct usb_device *device,
                           uint8_t *data,
                           uint16_t max_length);
static int usb_device_get_status(struct usb_device *device,
                                 struct usb_device_status *status);

static bool usb_is_configured(void);
static bool usb_is_tx_busy(void);

static struct usb_device_data usb_cdc_data;

static struct usb_device usb_cdc = {
    .name = "usb_cdc",
    .usb_init = usb_device_init,
    .usb_send_bytes = usb_device_send_bytes,
    .usb_send = usb_device_send,
    .usb_printf = usb_device_printf,
    .usb_available = usb_device_available,
    .usb_read = usb_device_read,
    .usb_get_status = usb_device_get_status,
    .priv_data = &usb_cdc_data,
};

static bool usb_is_configured(void)
{
    return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
           hUsbDeviceHS.pClassData != NULL;
}

static bool usb_is_tx_busy(void)
{
    USBD_CDC_HandleTypeDef *cdc_handle;

    if (!usb_is_configured()) {
        return false;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

    return cdc_handle->TxState != 0U;
}

static int usb_device_init(struct usb_device *device)
{
    struct usb_device_data *device_data;

    if (device == NULL || device->priv_data == NULL) {
        return USB_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;

    if (device_data->initialised) {
        return USB_DEVICE_OK;
    }

    device_data->rx_head = 0U;
    device_data->rx_tail = 0U;

    device_data->rx_byte_count = 0U;
    device_data->tx_byte_count = 0U;
    device_data->rx_overflow_count = 0U;
    device_data->tx_busy_count = 0U;
    device_data->tx_error_count = 0U;

    __DMB();

    device_data->initialised = true;

    return USB_DEVICE_OK;
}

static uint16_t usb_device_available(struct usb_device *device)
{
    struct usb_device_data *device_data;
    uint32_t head;
    uint32_t tail;
    uint32_t available;

    if (device == NULL || device->priv_data == NULL) {
        return 0U;
    }

    device_data = device->priv_data;

    head = device_data->rx_head;
    tail = device_data->rx_tail;

    available = head - tail;

    if (available > USB_BSP_RX_BUFFER_SIZE) {
        available = USB_BSP_RX_BUFFER_SIZE;
    }

    return (uint16_t)available;
}

static int usb_device_read(struct usb_device *device,
                           uint8_t *data,
                           uint16_t max_length)
{
    struct usb_device_data *device_data;
    uint32_t head;
    uint32_t tail;
    uint32_t available;
    uint32_t read_length;
    uint32_t read_index;
    uint32_t first_length;

    if (device == NULL ||
        device->priv_data == NULL ||
        data == NULL ||
        max_length == 0U) {
        return USB_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;

    if (!device_data->initialised) {
        if (usb_device_init(device) != USB_DEVICE_OK) {
            return USB_DEVICE_ERR_USB;
        }
    }

    head = device_data->rx_head;
    tail = device_data->rx_tail;
    available = head - tail;

    if (available == 0U) {
        return 0;
    }

    if (available > USB_BSP_RX_BUFFER_SIZE) {
        available = USB_BSP_RX_BUFFER_SIZE;
    }

    read_length = available;

    if (read_length > max_length) {
        read_length = max_length;
    }

    read_index = tail & (USB_BSP_RX_BUFFER_SIZE - 1U);

    first_length = USB_BSP_RX_BUFFER_SIZE - read_index;

    if (first_length > read_length) {
        first_length = read_length;
    }

    memcpy(data,
           &device_data->rx_buffer[read_index],
           first_length);

    if (read_length > first_length) {
        memcpy(&data[first_length],
               device_data->rx_buffer,
               read_length - first_length);
    }

    __DMB();

    device_data->rx_tail = tail + read_length;

    return (int)read_length;
}

static int usb_device_send_bytes(struct usb_device *device,
                                 const uint8_t *data,
                                 uint16_t length)
{
    struct usb_device_data *device_data;
    uint8_t result;

    if (device == NULL ||
        device->priv_data == NULL ||
        data == NULL ||
        length == 0U) {
        return USB_DEVICE_ERR_PARAM;
    }

    if (length > USB_BSP_TX_BUFFER_SIZE) {
        return USB_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;

    if (!device_data->initialised) {
        if (usb_device_init(device) != USB_DEVICE_OK) {
            return USB_DEVICE_ERR_USB;
        }
    }

    if (!usb_is_configured()) {
        return USB_DEVICE_ERR_NOT_READY;
    }

    if (usb_is_tx_busy()) {
        ++device_data->tx_busy_count;
        return USB_DEVICE_ERR_BUSY;
    }

    if (data != device_data->tx_buffer) {
        memcpy(device_data->tx_buffer, data, length);
    }

    result = CDC_Transmit_HS(device_data->tx_buffer, length);

    if (result == USBD_OK) {
        device_data->tx_byte_count += length;
        return USB_DEVICE_OK;
    }

    if (result == USBD_BUSY) {
        ++device_data->tx_busy_count;
        return USB_DEVICE_ERR_BUSY;
    }

    ++device_data->tx_error_count;

    return USB_DEVICE_ERR_USB;
}

static int usb_device_send(struct usb_device *device, const char *message)
{
    size_t length;

    if (message == NULL) {
        return USB_DEVICE_ERR_PARAM;
    }

    length = strlen(message);

    if (length == 0U ||
        length > USB_BSP_TX_BUFFER_SIZE ||
        length > UINT16_MAX) {
        return USB_DEVICE_ERR_PARAM;
    }

    return usb_device_send_bytes(device,
                                 (const uint8_t *)message,
                                 (uint16_t)length);
}

static int usb_device_printf(struct usb_device *device,
                             const char *format,
                             ...)
{
    struct usb_device_data *device_data;
    va_list arguments;
    int length;

    if (device == NULL ||
        device->priv_data == NULL ||
        format == NULL) {
        return USB_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;

    va_start(arguments, format);

    length = vsnprintf((char *)device_data->printf_buffer,
                       USB_BSP_PRINTF_BUFFER_SIZE,
                       format,
                       arguments);

    va_end(arguments);

    if (length <= 0 ||
        length >= (int)USB_BSP_PRINTF_BUFFER_SIZE) {
        return USB_DEVICE_ERR_PARAM;
    }

    return usb_device_send_bytes(device,
                                 device_data->printf_buffer,
                                 (uint16_t)length);
}

static int usb_device_get_status(struct usb_device *device,
                                 struct usb_device_status *status)
{
    struct usb_device_data *device_data;

    if (device == NULL ||
        device->priv_data == NULL ||
        status == NULL) {
        return USB_DEVICE_ERR_PARAM;
    }

    device_data = device->priv_data;

    status->initialised = device_data->initialised;
    status->configured = usb_is_configured();
    status->tx_busy = usb_is_tx_busy();
    status->rx_available = usb_device_available(device);

    __DMB();

    status->rx_byte_count = device_data->rx_byte_count;
    status->tx_byte_count = device_data->tx_byte_count;
    status->rx_overflow_count = device_data->rx_overflow_count;
    status->tx_busy_count = device_data->tx_busy_count;
    status->tx_error_count = device_data->tx_error_count;

    return USB_DEVICE_OK;
}

void usb_cdc_receive_data(const uint8_t *data, uint32_t length)
{
    struct usb_device_data *device_data = &usb_cdc_data;
    uint32_t head;
    uint32_t tail;
    uint32_t used;
    uint32_t free_length;
    uint32_t write_length;
    uint32_t write_index;
    uint32_t first_length;

    if (data == NULL || length == 0U) {
        return;
    }

    head = device_data->rx_head;
    tail = device_data->rx_tail;
    used = head - tail;

    if (used > USB_BSP_RX_BUFFER_SIZE) {
        used = USB_BSP_RX_BUFFER_SIZE;
    }

    free_length = USB_BSP_RX_BUFFER_SIZE - used;
    write_length = length;

    if (write_length > free_length) {
        write_length = free_length;
        device_data->rx_overflow_count += length - write_length;
    }

    if (write_length == 0U) {
        return;
    }

    write_index = head & (USB_BSP_RX_BUFFER_SIZE - 1U);

    first_length = USB_BSP_RX_BUFFER_SIZE - write_index;

    if (first_length > write_length) {
        first_length = write_length;
    }

    memcpy(&device_data->rx_buffer[write_index],
           data,
           first_length);

    if (write_length > first_length) {
        memcpy(device_data->rx_buffer,
               &data[first_length],
               write_length - first_length);
    }

    __DMB();

    device_data->rx_head = head + write_length;
    device_data->rx_byte_count += write_length;
}

struct usb_device *usb_get_device(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    if (strcmp(name, usb_cdc.name) != 0) {
        return NULL;
    }

    if (usb_device_init(&usb_cdc) != USB_DEVICE_OK) {
        return NULL;
    }

    return &usb_cdc;
}