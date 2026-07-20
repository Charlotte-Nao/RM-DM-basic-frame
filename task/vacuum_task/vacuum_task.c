//
// Created by charlotte on 7/20/26.
//

#include "vacuum_task.h"
#include <cmsis_os2.h>

void vacuum_task(void)
{
    Vacuum_System_PowerOn_Init();
    for (;;)
    {
        Vacuum_All_Update();
        osDelay(10);
    }
}