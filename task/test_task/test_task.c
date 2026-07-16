/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"
#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"

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

    osDelay(100U);
    yb_sd15m_set_target(servo_1,0,1000U);
    yb_sd15m_set_target(servo_2,0,1000U);
    yb_sd15m_set_target(servo_3,0,1000U);
    yb_sd15m_set_target(servo_4,0,1000U);

    osDelay(1500U);

    for (;;) {
        yb_sd15m_set_target(servo_1,0,1000U);
        yb_sd15m_set_target(servo_2,0,1000U);
        yb_sd15m_set_target(servo_3,0,1000U);
        yb_sd15m_set_target(servo_4,0,1000U);
        osDelay(1500U);

        yb_sd15m_set_target(servo_1,30,1000U);
        yb_sd15m_set_target(servo_2,30,1000U);
        yb_sd15m_set_target(servo_3,30,1000U);
        yb_sd15m_set_target(servo_4,30,1000U);

        osDelay(1500U);

    }
}
