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
    float now_pos_rad;
    float target_pos_rad;

    gm6020_yaw = motor_get_device("GM6020_YAW");
    while (gm6020_yaw == NULL || !motor_is_online(gm6020_yaw)) {
        osDelay(1000);
    }

        gm6020_yaw->send_enable_cmd(gm6020_yaw);
        osDelay(2000);

        gm6020_yaw->get_status(gm6020_yaw, "POS", &now_pos_rad);



    for (;;) {
        target_pos_rad = now_pos_rad + 0.5;
        gm6020_yaw->set_target(gm6020_yaw,1,target_pos_rad);
        osDelay(5000U);
        now_pos_rad = target_pos_rad;
    }

}
