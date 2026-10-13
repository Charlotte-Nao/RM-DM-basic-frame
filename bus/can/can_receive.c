/**
 * @file can_receive.c
 * @brief FDCAN FIFO0 receive and identifier-based dispatch.
 */

#include "can_receive.h"

#include "fdcan.h"

#include "../../bsp/can/can.h"
#include "../../device/motor/motor.h"
#include "../../device/motor/motor_internal.h"

static void can1_dispatch(uint32_t identifier, const uint8_t frame[8])
{
    struct motor_device *motor;

    switch (identifier) {
    case CAN_M3508_L_ID:
        motor = motor_get_device("M3508_L");
        if (motor == NULL) { return; }
        m3508_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_M3508_R_ID:
        motor = motor_get_device("M3508_R");
        if (motor == NULL) { return; }
        m3508_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    default:
        break;
    }
}
static void can2_dispatch(uint32_t identifier, const uint8_t frame[8])
{
    struct motor_device *motor;

    switch (identifier) {
    case CAN_DM8009P_LF_FEEDBACK_ID:
        motor = motor_get_device("DM8009P_LF");
        if (motor == NULL) { return; }
        dm8009p_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_DM_8009P_LB_FEEDBACK_ID:
        motor = motor_get_device("DM_8009P_LB");
        if (motor == NULL) { return; }
        dm8009p_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_DM_8009P_RF_FEEDBACK_ID:
        motor = motor_get_device("DM_8009P_RF");
        if (motor == NULL) { return; }
        dm8009p_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_DM_8009P_RB_FEEDBACK_ID:
        motor = motor_get_device("DM_8009P_RB");
        if (motor == NULL) { return; }
        dm8009p_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    default:
        break;
    }
}
static void can3_dispatch(uint32_t identifier, const uint8_t frame[8])
{
    struct motor_device *motor;

    switch (identifier) {
    case CAN_GM6020_YAW_ID:
        motor = motor_get_device("GM6020_YAW");
        if (motor == NULL) { return; }
        gm6020_feedback_calculate(motor, frame);
        motor->last_rx_tick = HAL_GetTick();
        break;

    case CAN_MG4005E_STIR_ID:
        motor = motor_get_device("MG4005E_STIR");
        if (motor == NULL) { return; }
        mg4005e_feedback_calculate(motor, frame);
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
        if (handle == &hfdcan3) {
            can3_dispatch(header.Identifier, frame);
        }
    }
}
