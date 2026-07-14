//
// Created by charlotte on 7/14/26.
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
        /* This is the only periodic motor update and CAN control-frame sender. */
        Motor_All_Update();

        next_wake_tick += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake_tick);
    }
}
