/**
 * @file test_task.c
 * @brief GM6020 CAN1 position--velocity closed-loop test.
 */

#include "../test_task/test_task.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/uart/uart.h"
#include "cmsis_os2.h"

#include "fdcan.h"
#include "../../application/global_data.h"
#include "../../device/motor/motor.h"

#define GM6020_DIAG_PERIOD_MS     500U
#define RAD_TO_DEG                 57.295779513082320876f
#define DEG_TO_RAD                 0.01745329251994329577f

static float pitch_to_gm6020_target_rad(float pitch_rad)
{
    float pitch_deg = pitch_rad * RAD_TO_DEG;
    float target_deg;

    /* BMI088 pitch is naturally -90...+90 degrees.  Map that continuous
     * 180-degree input range onto one complete GM6020 mechanical revolution. */
    if (pitch_deg > 90.0f) {
        pitch_deg = 90.0f;
    } else if (pitch_deg < -90.0f) {
        pitch_deg = -90.0f;
    }
    target_deg = (pitch_deg + 90.0f) * 1.0f;
    return target_deg * DEG_TO_RAD;
}

static void gm6020_print_feedback(struct uart_device *uart1,
                                  const struct motor_device *gm6020)
{
    float position_rad = 0.0f;
    float velocity_rpm = 0.0f;
    uint8_t temperature = 0U;
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};

    if (uart1 == NULL || gm6020 == NULL) {
        return;
    }

    gm6020->get_status(gm6020, "POS", &position_rad);
    gm6020->get_status(gm6020, "VEL", &velocity_rpm);
    gm6020->get_status(gm6020, "TEMP", &temperature);
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status);
    (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters);
    (void)uart1->uart_printf(uart1,
                             "CAN1: irq=%lu frame=%lu last_id=0x%03lX last_dlc=0x%08lX fifo=%lu tx_free=%lu TEC=%lu REC=%lu busoff=%lu LEC=%lu\r\n"
                             "GM6020: online=%u rx_tick=%lu pos=%.3f rad vel=%.1f rpm temp=%u C\r\n",
                             (unsigned long)g_can1_irq_count,
                             (unsigned long)g_can1_frame_count,
                             (unsigned long)g_can1_last_id,
                             (unsigned long)g_can1_last_dlc,
                             (unsigned long)HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0),
                             (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1),
                             (unsigned long)error_counters.TxErrorCnt,
                             (unsigned long)error_counters.RxErrorCnt,
                             (unsigned long)protocol_status.BusOff,
                             (unsigned long)protocol_status.LastErrorCode,
                             motor_is_online(gm6020) ? 1U : 0U,
                             (unsigned long)gm6020->last_rx_tick,
                             position_rad, velocity_rpm,
                             (unsigned int)temperature);
}

void test_task(void)
{
    struct motor_device *gm6020 = motor_get_device("GM6020_YAW");
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    uint32_t last_diag_tick = 0U;

    if (gm6020 == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    /* Do not enable before both CAN feedback and a valid IMU attitude are available. */
    while (!motor_is_online(gm6020) || global_data.imu_ready == 0U) {
        /* Keep sending a zero 0x1FF frame while waiting, without energising the motor. */
        Motor_All_Update();
        LED_SKY_SET();
        if ((osKernelGetTickCount() - last_diag_tick) >= GM6020_DIAG_PERIOD_MS) {
            last_diag_tick = osKernelGetTickCount();
            gm6020_print_feedback(uart1, gm6020);
        }
        osDelay(1U);
    }

    /* enable() synchronises the internal position target. */
    gm6020->send_enable_cmd(gm6020);

    for (;;) {
        float target_rad = pitch_to_gm6020_target_rad(global_data.imu_pitch_rad);

        /* -90/0/+90 IMU pitch maps continuously to 0/180/360 GM6020 degrees. */
        gm6020->set_target(gm6020, 1, (double)target_rad);
        Motor_All_Update();
        LED_RED_SET();
        if ((osKernelGetTickCount() - last_diag_tick) >= GM6020_DIAG_PERIOD_MS) {
            last_diag_tick = osKernelGetTickCount();
            gm6020_print_feedback(uart1, gm6020);
        }
        osDelay(1U);
    }
}
