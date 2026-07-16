/**
* @file servo_task.c
 * @brief YB-SD15M periodic update task.
 */

#include "servo_task.h"

#include "cmsis_os2.h"

#include "../../device/YB_SD15M/YB_SD15M.h"

void servo_task(void)
{
    YB_SD15M_System_PowerOn_Init();

    for (;;) {
        YB_SD15M_All_Update();

        osDelay(10);
    }
}
