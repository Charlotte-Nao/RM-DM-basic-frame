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
#include "../../bsp/LED/LED.h"
#include "../../bsp/usb/usb.h"

static int16_t servo_angle_to_command(float angle)
{
    if (angle >= 0.0f) {
        return (int16_t)(angle + 0.5f);
    }
    return (int16_t)(angle - 0.5f);
}

static void set_all_target(struct yb_sd15m_device *servo_1,
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
    set_all_target(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(1000U);

    Four_degree_of_freedom_calculation(&arm, b_above_pose, servo_angle);
    set_all_target(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(1000U);

    Four_degree_of_freedom_calculation(&arm, b_pose, servo_angle);
    set_all_target(servo_1, servo_2, servo_3, servo_4, servo_angle, 1000U);
    osDelay(2000U);
}

void test_task(void)
{
    struct usb_device *usb;
    struct yb_sd15m_device *servo_1;
    struct yb_sd15m_device *servo_2;
    struct yb_sd15m_device *servo_3;
    struct yb_sd15m_device *servo_4;
    struct vacuum_device *pump = vacuum_get_device("VACUUM_PUMP");
    struct vacuum_device *valve = vacuum_get_device("VACUUM_VALVE");

    usb = usb_get_device("usb_cdc");
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
    float aim_pose_array[4] = {0};
    uint8_t last_action = 0;
    float last_aim_pose_array[4] = {0};

    for (;;) {
        aim_pose_array[0] = aim_pose.x;
        aim_pose_array[1] = aim_pose.y;
        aim_pose_array[2] = aim_pose.z;
        aim_pose_array[3] = aim_pose.phi;

        if (memcmp(aim_pose_array,last_aim_pose_array,sizeof(aim_pose_array))!= 0)
        {
            if (Four_degree_of_freedom_calculation(&arm, aim_pose_array, servo_angle))
            {
                set_all_target(servo_1, servo_2, servo_3, servo_4,servo_angle, 1000U);
                LED_GREEN_SET();
            }
            else
            {
                usb->usb_printf(usb,"1");
                LED_RED_SET();
            }

        }

        if (aim_pose.action == 1 && last_action != 1)
        {
            pick_chess(pump,valve);
        }
        else if (aim_pose.action == 2 && last_action != 2)
        {
            put_chess(pump,valve);
        }
        else if (aim_pose.action == 3)
        {
            valve->disable(valve);
            pump->disable(pump);
        }

        last_action = aim_pose.action;
        memcpy(last_aim_pose_array,aim_pose_array,sizeof(aim_pose_array));

        osDelay(1);
    }
}