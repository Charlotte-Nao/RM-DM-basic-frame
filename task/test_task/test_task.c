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
#include "../../application/mode/mode_control.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/pwm/pwm.h"
#include "../../bsp/usb/usb.h"
#include "../../dsp/LinkageSolution/linkage_solution.h"
#include "../../dsp/LegLength/leg_length.h"
#include "../../dsp/State/state.h"
#include "../../dsp/LQR/lqr.h"
#include "../../dsp/ComprensiveControl/comprensive_control.h"
#include "../../dsp/TraceTracking/trace_tracking.h"

void test_task(void)
{
    osDelay(1000U);

    for (;;) {
        mode_control();
        vmc_calculate();
        leg_pd();
        state_update();
        // lqr_calculate();
        trace_tracking_calculate();
        equalize_phi0();
        torque_send();

        osDelay(1);
    }
}
