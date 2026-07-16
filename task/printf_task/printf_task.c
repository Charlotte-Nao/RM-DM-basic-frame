//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../bsp/uart/uart.h"
#include "../../application/global_data.h"

#define RAD_TO_DEG                    57.295779513082320876f
#define DEG_TO_RAD                    0.01745329251994329577f

void printf_task(void)
{
    struct motor_device *gm6020_pitch = motor_get_device("GM6020_PITCH");
    struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
    struct uart_device *uart1 = uart_get_device("uart1_dma");

    if (uart1 == NULL ) return;

    float position_rad = 0.0f;
    float velocity_rpm = 0.0f;
    uint8_t temperature = 0U;
    uint16_t encoder = 0U;

    for (;;)
    {
        if (gm6020_yaw == NULL) {(void)uart1->uart_printf(uart1,"GM6020_yaw未连接");}
        else
        {
            gm6020_yaw->get_status(gm6020_yaw, "POS", &position_rad);
            gm6020_yaw->get_status(gm6020_yaw, "VEL", &velocity_rpm);
            gm6020_yaw->get_status(gm6020_yaw, "TEMP", &temperature);
            gm6020_yaw->get_status(gm6020_yaw, "ENC", &encoder);

            (void)uart1->uart_printf(uart1,
            "GM6020_yaw: online=%u "
            "pos=%.3f "
            "rad vel=%.1f rpm "
            "temp=%u  "
            "encoder=%u\r\n",
            motor_is_online(gm6020_yaw) ? 1U : 0U,
            position_rad,
            velocity_rpm,
            (unsigned int)temperature,
            (unsigned int)encoder
            );
        }

        osDelay(200U);

        if (gm6020_pitch == NULL) {(void)uart1->uart_printf(uart1,"GM6020_pitch未连接");}
        else
        {
            gm6020_pitch->get_status(gm6020_pitch, "POS", &position_rad);
            gm6020_pitch->get_status(gm6020_pitch, "VEL", &velocity_rpm);
            gm6020_pitch->get_status(gm6020_pitch, "TEMP", &temperature);
            gm6020_pitch->get_status(gm6020_pitch, "ENC", &encoder);

            (void)uart1->uart_printf(uart1,
                     "GM6020_pitch: online=%u "
                     "pos=%.3f "
                     "rad vel=%.1f rpm "
                     "temp=%u  "
                     "encoder=%u\r\n",
                     motor_is_online(gm6020_yaw) ? 1U : 0U,
                     position_rad, velocity_rpm,
                     (unsigned int)temperature,
                     (unsigned int)encoder);

        }

        osDelay(200U);

        (void)uart1->uart_printf(uart1,
        "roll:%.2f,pitch:%.2f,yaw:%.2f,temp:%u\r\n",
        global_data.imu_roll_rad * RAD_TO_DEG,
        global_data.imu_pitch_rad * RAD_TO_DEG,
        global_data.imu_yaw_rad * RAD_TO_DEG,
        temperature);

        osDelay(200U);
    }
}