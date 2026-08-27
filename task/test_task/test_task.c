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
    struct motor_device *MG4005E;
    uint32_t next_wake_tick;

    MG4005E = motor_get_device("MG4005E_1");
    if (MG4005E == NULL) { LED_RED_SET(); return; }

    MG4005E->send_enable_cmd(MG4005E);
    osDelay(100U);

    next_wake_tick = osKernelGetTickCount();
    for (;;) {
        /* MG4005E speed target unit is dps. */
        MG4005E->set_target(MG4005E, 1, 0.0f);
        (void)osDelayUntil(next_wake_tick);
    }
}
