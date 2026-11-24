//
// Created by ASUS on 2026/9/4.
//

#ifndef DM_CAN_SEND_H
#define DM_CAN_SEND_H

#include <stdint.h>

void gimbal_send(int16_t ch3);
void gimbal_receive(const uint8_t frame[8]);
#endif //DM_CAN_SEND_H