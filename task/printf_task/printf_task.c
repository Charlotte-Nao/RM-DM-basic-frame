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

void printf_task(void)
{
    struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
    struct motor_device *m3508_2 = motor_get_device("M3508_2");
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    float position_rad = 0.0f;
    float target_pos_rad = 0.0f;

    if (uart1 == NULL) {
        return;
    }

    for (;;) {
        // if (gm6020_yaw != NULL) {
        //     gm6020_yaw->get_status(gm6020_yaw, "POS", &position_rad);
        //     gm6020_yaw->get_status(gm6020_yaw, "TARGET_POS", &target_pos_rad);
        //
        //     (void)uart1->uart_printf(uart1,
        //                              "%.4f,%.4f\r\n",
        //                              position_rad,
        //                              target_pos_rad);
        // }

        if (m3508_2 != NULL) {
            m3508_2->get_status(m3508_2, "POS", &position_rad);
            m3508_2->get_status(m3508_2, "TARGET_POS", &target_pos_rad);

            (void)uart1->uart_printf(uart1,
                                     "%.4f,%.4f\r\n",
                                     position_rad,
                                     target_pos_rad);
        }

        osDelay(20U);
    }
}