/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../dsp/calculation/calculation.h"

struct four_axis_robotic_arm arm = {
    .base_height = 107.5f,
    .upper_arm_length = 83.7f,
    .forearm_length = 121.5f,
    .wrist_length = 0.0f,
};

// float target_pose[4] = {
//     125.0f          ,  // x
//     93.0f,    // y
//     82.0f,  // z
//     0.0f,    // 末端 pitch 角度，单位 degree
// };

float target_pose[4] = {
    0.0f,  // x
    0.0f,    // y
    0.0f,  // z
    0.0f,    // 末端 pitch 角度，单位 degree
};

static int16_t servo_angle_to_command(float angle)
{
    if (angle >= 0.0f) {
        return (int16_t)(angle + 0.5f);
    }

    return (int16_t)(angle - 0.5f);
}

static void send_all_servo(struct yb_sd15m_device *servo_1,
                           struct yb_sd15m_device *servo_2,
                           struct yb_sd15m_device *servo_3,
                           struct yb_sd15m_device *servo_4,
                           const float servo_angle[4],
                           int move_time_ms)
{
    if (servo_1 == NULL || servo_2 == NULL || servo_3 == NULL || servo_4 == NULL ||
        servo_angle == NULL) {
        return;
    }

    yb_sd15m_set_target(servo_1, servo_angle_to_command(servo_angle[0]), move_time_ms);
    yb_sd15m_set_target(servo_2, servo_angle_to_command(servo_angle[1]), move_time_ms);
    yb_sd15m_set_target(servo_3, servo_angle_to_command(servo_angle[2]), move_time_ms);
    yb_sd15m_set_target(servo_4, servo_angle_to_command(servo_angle[3]), move_time_ms);
}

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

    // osDelay(100U);
    // yb_sd15m_set_target(servo_1,0,1000U);
    // yb_sd15m_set_target(servo_2,0,1000U);
    // yb_sd15m_set_target(servo_3,0,1000U);
    // yb_sd15m_set_target(servo_4,0,1000U);

    // osDelay(1500U);

    for (;;) {

         // Four_degree_of_freedom_calculation(&arm, target_pose, servo_angle);


        for (int i = 0; i < 4; i++)
        {
            servo_angle[i] = 15;
        }

        send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);

        osDelay(1500U);

        // memset(servo_angle, 30, 4 * sizeof(float));
        //
        // send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
        //
        // osDelay(1500U);

    }
}
