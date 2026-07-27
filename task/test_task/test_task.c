/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../device/motor/motor.h"
#include "../../device/vacuum/vacuum.h"
#include "../../dsp/calculation/calculation.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/usb/usb.h"

void test_task(void)
{
    struct motor_device *gm6020_yaw;
    float start_pos_rad;
    float end_pos_rad;

    gm6020_yaw = motor_get_device("GM6020_YAW");

    while (gm6020_yaw == NULL || !motor_is_online(gm6020_yaw)) {
        osDelay(100U);
    }

    gm6020_yaw->send_enable_cmd(gm6020_yaw);
    osDelay(100U);

    gm6020_yaw->get_status(gm6020_yaw, "POS", &start_pos_rad);

    /* 先选择一个固定的0.5 rad位移。 */
    end_pos_rad = start_pos_rad + 3.0f;

    for (;;) {
        gm6020_yaw->set_trace(gm6020_yaw, end_pos_rad, 0.02f);
        osDelay(2000U);

        gm6020_yaw->set_trace(gm6020_yaw, start_pos_rad, 0.02f);
        osDelay(2000U);
    }
}
