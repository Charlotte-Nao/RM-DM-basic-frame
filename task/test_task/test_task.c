/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../device/motor/motor.h"
#include "../../dsp/math.h"
#include "../../device/vacuum/vacuum.h"
#include "../../dsp/calculation/calculation.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/pwm/pwm.h"
#include "../../bsp/usb/usb.h"

void test_task(void)
{
    struct motor_device *dm_3507_1;

    float start_pos_rad;
    float end_pos_rad;

    dm_3507_1 = motor_get_device("DM3507_1");

    while (dm_3507_1 == NULL || !motor_is_online(dm_3507_1)) {
        osDelay(100U);
    }

    dm_3507_1->send_enable_cmd(dm_3507_1);
    osDelay(100U);

    dm_3507_1->get_status(dm_3507_1, "POS", &start_pos_rad);

    end_pos_rad = start_pos_rad;


    for (;;) {
        // end_pos_rad = start_pos_rad + host_data.roll * DEG_TO_RAD;

        end_pos_rad = start_pos_rad + 3.0f;

        dm_3507_1->set_target(dm_3507_1, 1, end_pos_rad);

        // if (host_data.action == 1U) {
        //     pwm_set_pulse_us(PWM_CHANNEL_1, 20000);
        // } else if (host_data.action == 0U) {
        //     pwm_set_pulse_us(PWM_CHANNEL_1, 0U);
        // }

        osDelay(2000U);

    }
}
