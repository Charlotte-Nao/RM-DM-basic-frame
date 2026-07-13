/**
 * @file test_task.c
 * @brief GM6020 CAN1 position--velocity closed-loop test.
 */

#include "../test_task/test_task.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/uart/uart.h"
#include "cmsis_os2.h"

#include "../../device/motor/motor.h"

/* Target mechanical angle.  Keep this small for the first powered test. */
#define GM6020_TEST_TARGET_RAD    0.5f
#define GM6020_DIAG_PERIOD_MS     500U

static void gm6020_print_feedback(struct uart_device *uart1,
                                  const struct motor_device *gm6020)
{
    float position_rad = 0.0f;
    float velocity_rpm = 0.0f;
    uint8_t temperature = 0U;

    if (uart1 == NULL || gm6020 == NULL) {
        return;
    }

    gm6020->get_status(gm6020, "POS", &position_rad);
    gm6020->get_status(gm6020, "VEL", &velocity_rpm);
    gm6020->get_status(gm6020, "TEMP", &temperature);
    (void)uart1->uart_printf(uart1,
                             "GM6020: online=%u rx_tick=%lu pos=%.3f rad vel=%.1f rpm temp=%u C\r\n",
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

    /* Do not enable before FDCAN1 has delivered a valid encoder feedback frame. */
    while (!motor_is_online(gm6020)) {
        /* Keep sending a zero 0x1FF frame while waiting, without energising the motor. */
        Motor_All_Update();
        LED_SKY_SET();
        if ((osKernelGetTickCount() - last_diag_tick) >= GM6020_DIAG_PERIOD_MS) {
            last_diag_tick = osKernelGetTickCount();
            gm6020_print_feedback(uart1, gm6020);
        }
        osDelay(1U);
    }

    /* enable() synchronises the internal position target; set the test target after it. */
    gm6020->send_enable_cmd(gm6020);
    gm6020->set_target(gm6020, 1, (double)GM6020_TEST_TARGET_RAD);

    for (;;) {
        /* GM6020 update: position PID -> desired velocity; velocity PID -> voltage.
         * Motor_All_Update then emits one complete CAN1 0x1FF group frame. */
        Motor_All_Update();
        LED_RED_SET();
        if ((osKernelGetTickCount() - last_diag_tick) >= GM6020_DIAG_PERIOD_MS) {
            last_diag_tick = osKernelGetTickCount();
            gm6020_print_feedback(uart1, gm6020);
        }
        osDelay(1U);
    }
}
