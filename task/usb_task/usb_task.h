//
// Created by charlotte on 7/21/26.
//

#ifndef DM_USB_TASK_H
#define DM_USB_TASK_H

#include <stdint.h>
#include "../../protocol/protocol.h"

extern struct protocol_data g_usb_received_data;

void usb_task(void);

#endif //DM_USB_TASK_H
