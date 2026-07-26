//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../bsp/uart/uart.h"
#include "../../application/global_data.h"
#include "../../dsp/calculation/calculation.h"
// void printf_task(void)
// {
//     struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
//     struct uart_device *uart1 = uart_get_device("uart1_dma");
//
//     if (uart1 == NULL ) return;
//
//     float position_rad = 0.0f;
//     float target_pos = 0.0f;
//
//     for (;;)
//     {
//         if (gm6020_yaw == NULL) {(void)uart1->uart_printf(uart1,"GM6020_yaw未连接");}
//         else
//         {
//             gm6020_yaw->get_status(gm6020_yaw, "POS", &position_rad);
//             gm6020_yaw->get_status(gm6020_yaw, "TARGET_POS", &target_pos);
//             (void)uart1->uart_printf(uart1,
//             "GM6020_yaw: online=%u "
//             "pos=%.3f rad "
//             "target_pos=%.3f rad"
//             "\r\n",
//             motor_is_online(gm6020_yaw) ? 1U : 0U,
//             position_rad,
//             target_pos
//             );
//         }
//
//         osDelay(200U);
//     }
// }


void printf_task(void)
{
    struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    float position_rad = 0.0f;
    float target_pos_rad = 0.0f;

    if (uart1 == NULL) {
        return;
    }

    for (;;) {
        if (gm6020_yaw != NULL) {
            gm6020_yaw->get_status(gm6020_yaw, "POS", &position_rad);
            gm6020_yaw->get_status(gm6020_yaw, "TARGET_POS", &target_pos_rad);

            (void)uart1->uart_printf(uart1,
                                     "%.4f,%.4f\r\n",
                                     position_rad,
                                     target_pos_rad);
        }

        osDelay(20U);
    }
}