//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../bsp/uart/uart.h"
#include "../../device/motor/motor.h"

#define PRINTF_TASK_PERIOD_MS 20U
#define DM3507_POSITION_LIMIT_RAD 12.566f
#define DM3507_POSITION_SPAN_RAD (2.0f * DM3507_POSITION_LIMIT_RAD)

static float dm3507_wrap_position_for_plot(float position_rad)
{
    while (position_rad > DM3507_POSITION_LIMIT_RAD) {
        position_rad -= DM3507_POSITION_SPAN_RAD;
    }
    while (position_rad < -DM3507_POSITION_LIMIT_RAD) {
        position_rad += DM3507_POSITION_SPAN_RAD;
    }
    return position_rad;
}

void printf_task(void)
{
    struct uart_device *uart10 = uart_get_device("uart10_dma");
    struct motor_device *dm3507 = motor_get_device("DM3507_1");
    uint32_t next_wake_tick = osKernelGetTickCount();
    float target_pos_rad;
    float feedback_pos_rad;
    float target_plot_pos_rad;

    if (uart10 == NULL) {
        return;
    }

    for (;;) {
        if (dm3507 == NULL) {
            dm3507 = motor_get_device("DM3507_1");
        }

        if (dm3507 != NULL) {
            target_pos_rad = 0.0f;
            feedback_pos_rad = 0.0f;
            dm3507->get_status(dm3507, "TARGET_POS", &target_pos_rad);
            dm3507->get_status(dm3507, "POS", &feedback_pos_rad);
            target_plot_pos_rad = dm3507_wrap_position_for_plot(target_pos_rad);

            /* VOFA+ text: channel 0 target plot position, channel 1 feedback position. */
            (void)uart10->uart_printf(uart10,
                                      "%.4f,%.4f\r\n",
                                      (double)target_plot_pos_rad,
                                      (double)feedback_pos_rad);
        }

        next_wake_tick += PRINTF_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);
    }
}
