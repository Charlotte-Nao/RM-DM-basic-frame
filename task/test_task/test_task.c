/**
* @file test_task.c
 * @brief just test
 */

#include "test_task.h"

#include "cmsis_os2.h"
#include "../../application/global_data.h"
#include "../../application/motor.h"
#include "../../application/move.h"
#include "../../bus/can/can_send.h"

void test_task(void)
{
    osDelay(1000U);
    while (global_data.imu_data.imu_ready == 0U)
    {
        osDelay(1U);
    }
    pitch_enable();
    // shooter_enable(); // 摩擦轮暂不使能

    for (;;) {
        if (global_data.imu_data.imu_ready != 0U)
        {
            gimbal_send();
            move();
        }

        osDelay(1);
    }
}
