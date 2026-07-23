//
// Created by charlotte on 7/23/26.
//

#include "cmsis_os2.h"
#include "motor_task.h"

#include "../../device/motor/motor.h"

#define MOTOR_TASK_PERIOD_MS  1U

void motor_task(void)
{
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        Motor_All_Update();

        next_wake_tick += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);
    }
}
