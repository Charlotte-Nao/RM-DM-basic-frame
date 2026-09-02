#ifndef DM_USB_H
#define DM_USB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum usb_device_result {
    USB_DEVICE_OK = 0,
    USB_DEVICE_ERR_PARAM = -1,
    USB_DEVICE_ERR_BUSY = -2,
    USB_DEVICE_ERR_NOT_READY = -3,
    USB_DEVICE_ERR_USB = -4,
};

struct usb_device;

struct usb_device_status {
    bool initialised;
    bool configured;
    bool tx_busy;

    uint16_t rx_available;

    uint32_t rx_byte_count;
    uint32_t tx_byte_count;
    uint32_t rx_overflow_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
};

struct usb_device {
    const char *name;
    int (*usb_init)(struct usb_device *device);
    int (*usb_send_bytes)(struct usb_device *device,const uint8_t *data,uint16_t length);
    int (*usb_send)(struct usb_device *device, const char *message);
    int (*usb_printf)(struct usb_device *device, const char *format, ...);
    uint16_t (*usb_available)(struct usb_device *device);
    int (*usb_read)(struct usb_device *device,uint8_t *data,uint16_t max_length);
    int (*usb_get_status)(struct usb_device *device,struct usb_device_status *status);
    void *priv_data;
};

struct usb_device *usb_get_device(const char *name);

void usb_cdc_receive_data(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* DM_USB_H */