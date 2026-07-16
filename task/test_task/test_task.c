/**
* @file test_task.c
 * @brief YB-SD15M movement test.
 */

#include "test_task.h"
#include "cmsis_os2.h"
#include "../../device/YB_SD15M/YB_SD15M.h"

void test_task(void)
{
    struct yb_sd15m_device *servo;

    servo = yb_sd15m_get_device("YB_SD15M_1");

    while (servo == NULL)
    {
        servo = yb_sd15m_get_device("YB_SD15M_1");
        osDelay(1000U);
    }

    osDelay(100U);
    yb_sd15m_set_target(servo,YB_SD15M_POSITION_CENTER,1000U);

    osDelay(1500U);

    for (;;) {
        yb_sd15m_set_target(servo,1500U,1000U);

        osDelay(1500U);

        yb_sd15m_set_target(servo,2500U,1000U);

        osDelay(1500U);

        yb_sd15m_set_target(servo,YB_SD15M_POSITION_CENTER,1000U);

        osDelay(1500U);
    }
}
