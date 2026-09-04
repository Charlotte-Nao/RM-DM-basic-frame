/**
 * @file can_receive.c
 * @brief FDCAN FIFO0 receive and identifier-based dispatch.
 */

#include "can_receive.h"

#include "fdcan.h"

#include "../../bsp/can/can.h"
#include "../../device/motor/motor.h"
#include "../../device/motor/motor_internal.h"

#define CAN_DM3507_1_FEEDBACK_ID   0x000U
#define CAN_DM4310_PITCH_FEEDBACK_ID 0x003U
#define CAN_DM8009P_1_FEEDBACK_ID   0x004U

static void can1_dispatch(uint32_t identifier, const uint8_t frame[8])
{
    struct motor_device *motor;

    switch (identifier) {
    case CAN_DM3507_1_FEEDBACK_ID:
        motor = motor_get_device("DM3507_1");
        dm3507_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_DM4310_PITCH_FEEDBACK_ID:
        motor = motor_get_device("DM4310_PITCH");
        dm4310_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_DM8009P_1_FEEDBACK_ID:
        motor = motor_get_device("DM8009P_1");
        dm8009p_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_MG4005E_ID:
        motor = motor_get_device("MG4005E_1");
        mg4005e_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_M3508_2_ID:
        motor = motor_get_device("M3508_2");
        m3508_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_GM6020_PITCH_ID:
        motor = motor_get_device("GM6020_PITCH");
        gm6020_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_GM6020_YAW_ID:
        motor = motor_get_device("GM6020_YAW");
        gm6020_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    default:
        break;
    }
}
static void can2_dispatch(uint32_t identifier, const uint8_t frame[8])
{
}
static void can3_dispatch(uint32_t identifier, const uint8_t frame[8])
{
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
        if (handle == &hfdcan3) {
            can3_dispatch(header.Identifier, frame);
        }
    }
}
