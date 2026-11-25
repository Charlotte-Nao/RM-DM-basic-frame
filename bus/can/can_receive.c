/**
 * @file can_receive.c
 * @brief FDCAN FIFO0 receive and identifier-based dispatch.
 */

#include "can_receive.h"

#include "fdcan.h"

#include "../../application/global_data.h"
#include "../../bsp/can/can.h"
#include "../../device/motor/motor.h"
#include "../../device/motor/motor_internal.h"

static void can1_dispatch(uint32_t identifier, const uint8_t frame[8])
{
    switch (identifier) {
    case CAN_GIMBAL_COMMAND_ID:
        // 底盘0x25前2字节为有符号16位大端序ch3，其余字节暂不使用。
        global_data.gimbal_data.pitch_ch3 = (int16_t)(((uint16_t)frame[0] << 8) | frame[1]);
        break;

    default:
        break;
    }
}

static void can2_dispatch(uint32_t identifier, const uint8_t frame[8])
{
    struct motor_device *motor;

    switch (identifier) {
    case CAN_DM4310_PITCH_FEEDBACK_ID:
        motor = motor_get_device("DM4310_PITCH");
        if (motor == NULL) { return; }
        dm4310_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_M3508_SHOOT_L_ID:
        motor = motor_get_device("M3508_SHOOT_L");
        if (motor == NULL) { return; }
        m3508_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_M3508_SHOOT_R_ID:
        motor = motor_get_device("M3508_SHOOT_R");
        if (motor == NULL) { return; }
        m3508_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    default:
        break;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t frame[8];

    if ((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame) != HAL_OK) {
            break;
        }

        if (header.IdType != FDCAN_STANDARD_ID || header.DataLength != FDCAN_DLC_BYTES_8) {
            continue;
        }

        if (handle == &hfdcan1) {
            can1_dispatch(header.Identifier, frame);
        }
        else if (handle == &hfdcan2) {
            can2_dispatch(header.Identifier, frame);
        }
    }
}
