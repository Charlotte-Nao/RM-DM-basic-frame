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

    end_pos_rad = start_pos_rad;


    for (;;) {
        end_pos_rad = start_pos_rad + host_data.roll * DEG_TO_RAD;
        gm6020_yaw->set_target(gm6020_yaw, 1, end_pos_rad);

        if (host_data.action == 1U) {
            pwm_set_pulse_us(PWM_CHANNEL_1, 20000);
        } else if (host_data.action == 0U) {
            pwm_set_pulse_us(PWM_CHANNEL_1, 0U);
        }

        osDelay(1U);

    }
}
