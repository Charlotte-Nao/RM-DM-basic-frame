/**
* @file servo_task.c
 * @brief YB-SD15M periodic update task.
 */

#include "servo_task.h"

#include "cmsis_os2.h"

#include "../../device/YB_SD15M/YB_SD15M.h"

#define SERVO_TASK_PERIOD_MS    10U

void servo_task(void)
{
    int init_result;

    /*
     * 获取 UART7、注册接收回调并初始化所有 YB-SD15M 实例。
     * 整个系统只调用一次。
     */
    init_result = YB_SD15M_System_PowerOn_Init();

    if (init_result != YB_SD15M_OK) {
        /*
         * 初始化失败时停留在此处。
         * 后续可以增加串口日志或 LED 报警。
         */
        for (;;) {
            osDelay(1000U);
        }
    }

    for (;;) {
        /*
         * 检查是否存在新目标。
         * 有新目标则下发位置控制帧；
         * 没有新目标则按照设备层设定周期查询反馈。
         */
        YB_SD15M_All_Update();

        osDelay(SERVO_TASK_PERIOD_MS);
    }
}