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

void test_task(void)
{
    struct motor_device *gm6020_pitch = motor_get_device("GM6020_PITCH");
    struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    uint32_t last_diag_tick = 0U;

    if (gm6020_pitch == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    if (gm6020_yaw == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    /* Do not enable before both CAN feedback and a valid IMU attitude are available. */
    while (!motor_is_online(gm6020_pitch) || global_data.imu_ready == 0U) {
        LED_SKY_SET();
        osDelay(1U);
    }

    /* enable() synchronises the internal position target. */
    gm6020_pitch->send_enable_cmd(gm6020_pitch);
    gm6020_yaw->send_enable_cmd(gm6020_yaw);

    for (;;) {
        float target_rad = 0;
        float angle_rad_differ = global_data.imu_pitch_rad;
        uint16_t now_encoder = 0;
        gm6020_pitch->get_status(gm6020_pitch, "ENC", &now_encoder);
        float encoder_target = now_encoder + angle_rad_differ * 8192 / 2 / 3.1415926;
        if (encoder_target > 8192)
        {
            encoder_target = encoder_target - 8192;
        }
        if (encoder_target < 0)
        {
            encoder_target = encoder_target + 8192;
        }
        target_rad = encoder_target * 2 * 3.1415926 / 8192;

        /* -90/0/+90 IMU pitch maps continuously to 0/180/360 GM6020 degrees. */
        gm6020_pitch->set_target(gm6020_pitch, 1, (double)target_rad);

        angle_rad_differ = global_data.imu_yaw_rad;
        gm6020_yaw->get_status(gm6020_yaw, "ENC", &now_encoder);
        encoder_target = now_encoder - angle_rad_differ * 8192 / 2 / 3.1415926;
        if (encoder_target > 8192)
        {
            encoder_target = encoder_target - 8192;
        }
        if (encoder_target < 0)
        {
            encoder_target = encoder_target + 8192;
        }
        target_rad = encoder_target * 2 * 3.1415926 / 8192;

        /* -90/0/+90 IMU pitch maps continuously to 0/180/360 GM6020 degrees. */
        gm6020_yaw->set_target(gm6020_yaw, 1, (double)target_rad);


        LED_RED_SET();
        osDelay(1U);
    }
}
