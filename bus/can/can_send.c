//
// Created by ASUS on 2026/9/4.
//

#include "can_send.h"
#include "fdcan.h"
#include "../../application/global_data.h"
#include "../../bsp/can/can.h"
#include <string.h>

void gimbal_send(void)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    float yaw = global_data.imu_data.imu_yaw_rad;

    header.Identifier = CAN_GIMBAL_YAW_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    // 0x12前4字节为小端float型Yaw，单位rad，其余4字节补0。
    memcpy(data, &yaw, sizeof(yaw));
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}