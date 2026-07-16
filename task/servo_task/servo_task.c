/**
* @file servo_task.c
 * @brief YB-SD15M periodic update task.
 */

#include "servo_task.h"

#include "cmsis_os2.h"

#include "../../device/YB_SD15M/YB_SD15M.h"

void servo_task(void)
{
    /*
     * 获取 UART7、注册接收回调并初始化所有 YB-SD15M 实例。
     * 整个系统只调用一次。
     */
    YB_SD15M_System_PowerOn_Init();

    for (;;) {
        /*
         * 检查是否存在新目标。
         * 有新目标则下发位置控制帧；
         * 没有新目标则按照设备层设定周期查询反馈。
         */
        YB_SD15M_All_Update();

        osDelay(10);
    }
}
