//
// Created by ASUS on 2026/9/4.
//

#include "can_send.h"
#include "fdcan.h"
#include "../../application/global_data.h"
#include <string.h>

void gimbal_send(int16_t ch3)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint16_t pitch_channel = (uint16_t)ch3;

    header.Identifier = 0x25U;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    // ch3按有符号16位大端序发送，其余6字节补0。
    data[0] = (uint8_t)(pitch_channel >> 8);
    data[1] = (uint8_t)pitch_channel;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &header, data);
}

void gimbal_receive(const uint8_t frame[8])
{
    float yaw;

    // 0x12前4字节为小端float，单位rad，其余4字节保留。
    memcpy(&yaw, frame, sizeof(yaw));
    global_data.gimbal_data.gimbal_yaw_rad = yaw;
    global_data.gimbal_data.gimbal_yaw_ready = 1U;
}