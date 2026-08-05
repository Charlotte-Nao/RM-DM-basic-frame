//
// Created by charlotte on 7/21/26.
//

#include "./usb_task.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include "../../bsp/usb/usb.h"
#include "../../protocol/protocol.h"
#include"../../application/global_data.h"

void usb_task(void)
{
    struct usb_device *usb;
    uint8_t rx_buffer[128];
    int result;
    usb = usb_get_device("usb_cdc");

    if (usb == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    for (;;)
    {
        result = usb->usb_read(usb, rx_buffer, sizeof(rx_buffer));
        if (result > 0) {

        }
        osDelay(1U);
    }
}