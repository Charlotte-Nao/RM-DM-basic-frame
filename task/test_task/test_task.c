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
#include "../../bsp/usb/usb.h"

void test_task(void)
{
    struct motor_device *gm6020_yaw;
    struct motor_device *m3508_2;
    float start_pos_rad;
    float end_pos_rad;

    gm6020_yaw = motor_get_device("GM6020_YAW");
    m3508_2 = motor_get_device("M3508_2");

    while (gm6020_yaw == NULL || !motor_is_online(gm6020_yaw)) {
        osDelay(100U);
    }
    //
    // while (m3508_2 == NULL || !motor_is_online(m3508_2)) {
    //     osDelay(100U);
    // }

    // m3508_2->send_enable_cmd(m3508_2);
    gm6020_yaw->send_enable_cmd(gm6020_yaw);
    osDelay(100U);

    // m3508_2->get_status(m3508_2,"POS", &start_pos_rad);
    gm6020_yaw->get_status(gm6020_yaw, "POS", &start_pos_rad);

    /* 先选择一个固定的0.5 rad位移。 */
    end_pos_rad = start_pos_rad + 3.0f;


    for (;;) {
        // set_trace函数
        gm6020_yaw->set_trace(gm6020_yaw, end_pos_rad, 0.02f);
        osDelay(2000U);

        gm6020_yaw->set_trace(gm6020_yaw, start_pos_rad, 0.02f);
        osDelay(2000U);

        // m3508_2->set_trace(m3508_2,end_pos_rad, 0.02f);
        // osDelay(2000U);
        //
        // m3508_2->set_trace(m3508_2, start_pos_rad, 0.02f);
        // osDelay(2000U);

        // 位置阶跃响应：target_pos直接跳变，速度/加速度前馈为0。
        // gm6020_yaw->set_target(gm6020_yaw, 3, end_pos_rad, 0.0f, 0.0f);
        // osDelay(2000U);
        //
        // gm6020_yaw->set_target(gm6020_yaw, 3, start_pos_rad, 0.0f, 0.0f);
        // osDelay(2000U);

        // const uint32_t period_ms = 10U;
        // const uint32_t duration_ms = 2000U;
        // const float target_velocity_rad_s = 1.0f;
        // const float target_velocity_rpm = target_velocity_rad_s * RAD_S_TO_RPM;
        //
        // for (uint32_t elapsed_ms = 0U;
        //      elapsed_ms <= duration_ms;
        //      elapsed_ms += period_ms) {
        //     float elapsed_s = (float)elapsed_ms * 0.001f;
        //     float target_pos_rad =
        //         start_pos_rad + target_velocity_rad_s * elapsed_s;
        //
        //     gm6020_yaw->set_target(gm6020_yaw,3,target_pos_rad,target_velocity_rpm,.0f);
        //     osDelay(period_ms);
        // }
        //
        // gm6020_yaw->set_target(gm6020_yaw, 3, start_pos_rad, 0.0f, 0.0f);
        // osDelay(1000U);

        // 恒定加速度响应：target_pos按0.5*a*t^2推进，target_vel按a*t增加。
        // const uint32_t period_ms = 10U;
        // const uint32_t duration_ms = 2000U;
        // const float target_acceleration_rad_s2 = 1.0f;
        //
        // for (uint32_t elapsed_ms = 0U;
        //      elapsed_ms <= duration_ms;
        //      elapsed_ms += period_ms) {
        //     float elapsed_s = (float)elapsed_ms * 0.001f;
        //     float target_velocity_rad_s =
        //         target_acceleration_rad_s2 * elapsed_s;
        //     float target_pos_rad =
        //         start_pos_rad +
        //         0.5f * target_acceleration_rad_s2 * elapsed_s * elapsed_s;
        //
        //     gm6020_yaw->set_target(
        //         gm6020_yaw,
        //         3,
        //         target_pos_rad,
        //         target_velocity_rad_s * RAD_S_TO_RPM,
        //         target_acceleration_rad_s2);
        //     osDelay(period_ms);
        // }
        //
        // gm6020_yaw->set_target(gm6020_yaw, 3, start_pos_rad, 0.0f, 0.0f);
        // osDelay(1000U);



    }
}
