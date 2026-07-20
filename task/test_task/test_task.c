/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../dsp/calculation/calculation.h"
#include "../../application/global_data.h"



// 静态返回地址
// float target_pose[4] = {
//     160.0f,  // x
//     0.0f,    // y
//     180.0f,  // z
//     75.0f,    // 末端 pitch 角度，单位 degree
// };

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

    float target_pose[4] = {
        255.0f,
        - 35.0f,
        65.0f,
        75.0f,
    };

    float zero_pose[4] =    {160.0f,    0.0f,    180.0f, 75.0f};

    float one_pose[4] =     {255.0f,    35.0f,   85.0f, 75.0f};
    float two_pose[4] =     {255.0f,    0.0f,    85.0f, 75.0f};
    float three_pose[4] =   {255.0f,    - 35.0f, 85.0f, 75.0f};

    float four_pose[4] =    {225.0f,    30.0f,   70.0f, 75.0f};
    float five_pose[4] =    {225.0f,    0.0f,    70.0f, 75.0f};
    float six_pose[4] =     {225.0f,    - 30.0f, 70.0f, 75.0f};

    float seven_pose[4] =   {200.0f,    35.0f,   70.0f, 75.0f};
    float eight_pose[4] =   {200.0f,    0.0f,    70.0f, 75.0f};
    float nine_pose[4] =    {202.0f,    - 35.0f, 70.0f, 75.0f};

    float *board_pose_list[10] = {
        zero_pose,
        one_pose,
        two_pose,
        three_pose,
        four_pose,
        five_pose,
        six_pose,
        seven_pose,
        eight_pose,
        nine_pose,
    };

    int temp_i = 0 ;

    for (;;) {

        // Four_degree_of_freedom_calculation(&arm, target_pose, servo_angle);
        Four_degree_of_freedom_calculation(&arm, board_pose_list[temp_i], servo_angle);
        // Four_degree_of_freedom_calculation(&arm, board_pose_list[5], servo_angle);
        // Four_degree_of_freedom_calculation(&arm, board_pose_list[temp_i], servo_angle);
        temp_i ++ ;
        temp_i = temp_i % 10;

        send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);

        osDelay(3000U);

        // for (int i = 0; i < 4; i++)
        // {
        //     servo_angle[i] = 30;
        // }

        // servo_angle[0] = 0;
        // servo_angle[1] = 20;
        // servo_angle[2] = 30;
        // servo_angle[3] = 20;

        Four_degree_of_freedom_calculation(&arm, board_pose_list[0], servo_angle);
        send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);

        osDelay(3000U);

    }
}
