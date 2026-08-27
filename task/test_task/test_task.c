/**
* @file test_task.c
 * @brief just test
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../dsp/math.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/pwm/pwm.h"
#include "../../bsp/usb/usb.h"

void test_task(void)
{
    // struct motor_device *DM_8009P;
    // DM_8009P = motor_get_device("DM8009P_1");
    // if (DM_8009P == NULL) { LED_RED_SET(); return; }
    // DM_8009P->send_enable_cmd(DM_8009P);
    //
    // osDelay(100U);
    // DM_8009P->set_target(DM_8009P, 1, 1.5f);
    // uint32_t next_wake_tick;
    // next_wake_tick = osKernelGetTickCount();
    for (;;) {
        // next_wake_tick += 2000U;
        // (void)osDelayUntil(next_wake_tick);
        // DM_8009P->set_target(DM_8009P, 1, -1.5f);
        // next_wake_tick += 2000U;
        // (void)osDelayUntil(next_wake_tick);
        // DM_8009P->set_target(DM_8009P, 1, 1.5f);
    }
}
