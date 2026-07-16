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
    int result;

    /*
     * 获取在 YB_SD15M.c 中实例化的物理舵机：
     *
     * static struct yb_sd15m_device yb_sd15m_1
     */
    servo = yb_sd15m_get_device("YB_SD15M_1");

    if (servo == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    /*
     * FreeRTOS 各任务启动顺序不完全确定。
     * 等待 servo_task 完成设备层初始化。
     */
    do {
        result = yb_sd15m_set_target(
            servo,
            YB_SD15M_POSITION_CENTER,
            1000U
        );

        if (result == YB_SD15M_ERR_NOT_READY) {
            osDelay(10U);
        }
    } while (result == YB_SD15M_ERR_NOT_READY);

    if (result != YB_SD15M_OK) {
        for (;;) {
            osDelay(1000U);
        }
    }

    /*
     * 等待第一次动作完成。
     */
    osDelay(1500U);

    for (;;) {

        (void)yb_sd15m_set_target(
            servo,
            1500U,
            1000U
        );

        osDelay(1500U);

        /*
         * 向另一个方向运动。
         */
        (void)yb_sd15m_set_target(
            servo,
            2500U,
            1000U
        );

        osDelay(1500U);

        /*
         * 返回中位。
         */
        (void)yb_sd15m_set_target(
            servo,
            YB_SD15M_POSITION_CENTER,
            1000U
        );

        osDelay(1500U);
    }
}