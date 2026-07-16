/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"
#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../dsp/calculation/calculation.h"

struct four_axis_robotic_arm arm = {
    .base_height = 100.0f,
    .upper_arm_length = 120.0f,
    .forearm_length = 120.0f,
    .wrist_length = 60.0f,
};

float target_pose[4] = {
    150.0f,  // x
    0.0f,    // y
    120.0f,  // z
    0.0f,    // 末端 pitch 角度，单位 degree
};

void test_task(void)
{
    struct yb_sd15m_device *servo_1;
    struct yb_sd15m_device *servo_2;
    struct yb_sd15m_device *servo_3;
    struct yb_sd15m_device *servo_4;

    servo_1 = yb_sd15m_get_device("YB_SD15M_1");
    servo_2 = yb_sd15m_get_device("YB_SD15M_2");
    servo_3 = yb_sd15m_get_device("YB_SD15M_3");
    servo_4 = yb_sd15m_get_device("YB_SD15M_4");

    while (servo_1 == NULL || servo_2 == NULL || servo_3 == NULL || servo_4 == NULL)
    {
        servo_1 = yb_sd15m_get_device("YB_SD15M_1");
        servo_2 = yb_sd15m_get_device("YB_SD15M_2");
        servo_3 = yb_sd15m_get_device("YB_SD15M_3");
        servo_4 = yb_sd15m_get_device("YB_SD15M_4");
        osDelay(1000U);
    }

    float servo_angle[4] = {0};

    osDelay(100U);
    yb_sd15m_set_target(servo_1,0,1000U);
    yb_sd15m_set_target(servo_2,0,1000U);
    yb_sd15m_set_target(servo_3,0,1000U);
    yb_sd15m_set_target(servo_4,0,1000U);

    osDelay(1500U);

    for (;;) {

        Four_degree_of_freedom_calculation(&arm, target_pose, servo_angle);

        osDelay(1500U);

    }
}
