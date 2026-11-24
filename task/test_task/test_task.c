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
#include "../../dsp/big_jump/big_jump.h"
#include "../../dsp/LinkageSolution/linkage_solution.h"
#include "../../dsp/LegLength/leg_length.h"
#include "../../dsp/State/state.h"
#include "../../dsp/LQR/lqr.h"
#include "../../dsp/ComprensiveControl/comprensive_control.h"
#include "../../dsp/FnSolve/fn_solve.h"
#include "../../dsp/big_jump/big_jump.h"
#include "../../dsp/climb/climb.h"
#include "../../dsp/self_save/self_save.h"
#include "../../dsp/TraceTracking/trace_tracking.h"
#include "../../dsp/Gimbal/gimbal.h"

void test_task(void)
{
    osDelay(1000U);

    for (;;) {
        mode_control();
        vmc_calculate();
        leg_pd();
        state_update();
        fn_solve();
        self_save();
        if (global_data.save_stage == 1U)
        {
            jump_air();
            // lqr_calculate();
            if (global_data.climb_stage == 0U || global_data.climb_stage == 1U || global_data.save_stage == 1U)
            {
                trace_tracking_calculate();
            }

            if (global_data.jump_stage != 3U && global_data.jump_stage != 4U)
            {
                equalize_phi0();
                // equalize_roll();
                steering_control();
            }
            climb();
        }
        torque_send ();
        gimbal_select();

        osDelay(1);
    }

}