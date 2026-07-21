/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../device/vacuum/vacuum.h"
#include "../../dsp/calculation/calculation.h"
#include "../../application/global_data.h"

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

static void pick_chess(struct vacuum_device *pump, struct vacuum_device *valve)
{
    valve->disable(valve);
    pump->enable(pump);
}

static void put_chess(struct vacuum_device *pump, struct vacuum_device *valve)
{
    valve->enable(valve);
    pump->disable(pump);
}

static void move_a_to_b(struct yb_sd15m_device *servo_1,
                        struct yb_sd15m_device *servo_2,
                        struct yb_sd15m_device *servo_3,
                        struct yb_sd15m_device *servo_4,
                        const float a_pose[4],
                        const float b_pose[4])
{
    float a_above_pose[4];
    float b_above_pose[4];
    float servo_angle[4] = {0};

    if (a_pose == NULL || b_pose == NULL) {
        return;
    }

    memcpy(a_above_pose, a_pose, sizeof(a_above_pose));
    memcpy(b_above_pose, b_pose, sizeof(b_above_pose));
    if (a_above_pose[2] < 120 ){   a_above_pose[2] += 30.0f;}
    if (b_above_pose[2] < 120 ){   b_above_pose[2] += 30.0f;}

    Four_degree_of_freedom_calculation(&arm, a_above_pose, servo_angle);
    send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(1000U);

    Four_degree_of_freedom_calculation(&arm, b_above_pose, servo_angle);
    send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(1000U);

    Four_degree_of_freedom_calculation(&arm, b_pose, servo_angle);
    send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(2000U);
}

void test_task(void)
{
    struct yb_sd15m_device *servo_1;
    struct yb_sd15m_device *servo_2;
    struct yb_sd15m_device *servo_3;
    struct yb_sd15m_device *servo_4;
    struct vacuum_device *pump = vacuum_get_device("VACUUM_PUMP");
    struct vacuum_device *valve = vacuum_get_device("VACUUM_VALVE");

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

    float zero_pose[4] =    {160.0f,    0.0f,    180.0f, 75.0f};
    float one_pose[4] =     {250.0f,    15.0f,    75.0f, 75.0f};
    float two_pose[4] =     {250.0f,    - 25.0f, 75.0f, 75.0f};
    float three_pose[4] =   {245.0f,    - 55.0f, 75.0f, 75.0f};
    float four_pose[4] =    {225.0f,    15.0f,   65.0f, 75.0f};
    float five_pose[4] =    {225.0f,    -25.0f,    65.0f, 75.0f};
    float six_pose[4] =     {225.0f,    - 55.0f, 62.0f, 75.0f};
    float seven_pose[4] =   {200.0f,    20.0f,   62.0f, 75.0f};
    float eight_pose[4] =   {200.0f,    -20.0f,    62.0f, 75.0f};
    float nine_pose[4] =    {200.0f,    - 55.0f, 62.0f, 75.0f};

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

    float black_one[4]   =    {250.0f,   60.0f,  80.0f,  75.0f};
    float black_two[4]   =    {225.0f,   65.0f,  65.0f,  75.0f};
    float black_three[4] =    {200.0f,   70.0f,  60.0f,  75.0f};
    float black_four[4]  =   {175.0f,    75.0f,  60.0f,  75.0f};
    float black_five[4]  =   {140.0f,    85.0f,  60.0f,  75.0f};
    float white_one[4]   =  {240.0f,    - 105.0f,75.0f,  75.0f};
    float white_two[4]   =  {220.0f,    - 105.0f,    67.0f,  75.0f};
    float white_three[4] =  {190.0f ,   - 110.0f,    65.0f,  75.0f};
    float white_four[4]  =  {165.0f ,   - 115.0f,    60.0f,  75.0f};
    float white_five[4]  =  {135.0f ,   - 120.0f,   60.0f,  75.0f};

    float *chess_pose_list[10] = {
        black_one,
        black_two,
        black_three,
        black_four,
        black_five,
        white_one,
        white_two,
        white_three,
        white_four,
        white_five,
    };
    float target_pose[4] = {
        250.0f,
        -80.0f,
        85.0f,
        75.0f,
    };
    int temp_i = 1 ;
    int temp_j = 0;
    Four_degree_of_freedom_calculation(&arm, board_pose_list[0], servo_angle);
    send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(2000U);

    for (;;) {

        for (int i = 0 ;i <4 ;i ++)
        {
            servo_angle[i] = 0;
        }
        send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
        osDelay(1000U);



        // Four_degree_of_freedom_calculation(&arm, target_pose, servo_angle);
        // Four_degree_of_freedom_calculation(&arm, board_pose_list[9], servo_angle);
        // Four_degree_of_freedom_calculation(&arm, board_pose_list[9], servo_angle);
        //
        // Four_degree_of_freedom_calculation(&arm , chess_pos_list[9], servo_angle);
        // send_all_servo(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
        // osDelay(2000U);

        // move_a_to_b(servo_1, servo_2, servo_3, servo_4,board_pose_list[0], chess_pose_list[temp_j]);
        // pick_chess(pump,valve);
        // osDelay(2000U);
        //
        // move_a_to_b(servo_1,servo_2,servo_3,servo_4,chess_pose_list[temp_j],board_pose_list[temp_i]);
        // put_chess(pump,valve);
        // osDelay(2000U);
        //
        // move_a_to_b(servo_1,servo_2,servo_3,servo_4,board_pose_list[temp_i],board_pose_list[0]);
        //
        // temp_j++;
        // temp_j = temp_j % 10;
        //
        // temp_i ++ ;
        // temp_i = temp_i % 10;
        // if (temp_i == 0)
        // {
        //     temp_i ++;
        // }

    }
}
