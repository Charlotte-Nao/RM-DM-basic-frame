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
    struct protocol_data received_data;
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
            if (protocol_parse(rx_buffer, (uint16_t)result, &received_data) > 0) {
                aim_pose.x = received_data.x;
                aim_pose.y = received_data.y;
                aim_pose.z = received_data.z;
                aim_pose.roll = (float)received_data.roll / 10.0f;
                aim_pose.action= received_data.action;
            }
        }
        osDelay(1U);
    }
}