//
// Created by charlotte on 7/21/26.
//

#include "./usb_task.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include "../../bsp/usb/usb.h"

#define USB_TEST_RX_BUFFER_SIZE 128U
#define USB_TEST_REPORT_PERIOD_MS 1000U
#define USB_TEST_TASK_PERIOD_MS 2U

void usb_task(void)
{
    struct usb_device *usb;
    uint8_t rx_buffer[USB_TEST_RX_BUFFER_SIZE];
    uint16_t pending_length = 0U;
    uint32_t last_report_tick = 0U;
    int result;

    usb = usb_get_device("usb_cdc");

    if (usb == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    for (;;) {
        if (pending_length > 0U) {
            result = usb->usb_send_bytes(usb, rx_buffer, pending_length);

            if (result == USB_DEVICE_OK) {
                pending_length = 0U;
            }
        } else {
            result = usb->usb_read(usb,
                                   rx_buffer,
                                   sizeof(rx_buffer));

            if (result > 0) {
                pending_length = (uint16_t)result;
            }
        }

        if (pending_length == 0U &&
            osKernelGetTickCount() - last_report_tick >=
            USB_TEST_REPORT_PERIOD_MS) {
            result = usb->usb_send(usb, "USB CDC alive\r\n");

            if (result == USB_DEVICE_OK) {
                last_report_tick = osKernelGetTickCount();
            }
            }

        osDelay(USB_TEST_TASK_PERIOD_MS);
    }
}
