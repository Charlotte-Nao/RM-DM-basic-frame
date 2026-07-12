/**
 * @file can.c
 * @brief Classic CAN filter and start-up configuration.
 *
 * STM32H723 names its CAN peripheral FDCAN, but the CubeMX project configures
 * it as FDCAN_FRAME_CLASSIC, i.e. ordinary CAN 2.0 frames rather than CAN FD.
 */

#include "can.h"

#include "fdcan.h"
#include "main.h"

static void can_start(FDCAN_HandleTypeDef *handle)
{
    FDCAN_FilterTypeDef filter = {0};

    /* Standard-ID mask filter: ID mask 0 accepts all standard CAN identifiers. */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(handle,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(handle) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        Error_Handler();
    }
}

void can_init(void)
{
    can_start(&hfdcan1);
    can_start(&hfdcan2);
    can_start(&hfdcan3);
}
