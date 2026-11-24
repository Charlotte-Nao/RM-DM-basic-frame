//
// Created by ASUS on 2026/11/22.
//

#include "self_save.h"
#include "../../application/global_data.h"
#include "../../dsp/climb/climb.h"

void self_save()
{
    if (global_data.self_stage == 2U )
    {
        global_data.control_data.T_w_l = 0.0f;
        global_data.control_data.T_w_r = 0.0f;
        global_data.control_data.T_p_l = 0.0f;
        global_data.control_data.T_p_r = 0.0f;
        if (global_data.middle_data.L_l < 0.20f && global_data.middle_data.L_r < 0.20f)
        {
            global_data.save_stage = 3U;
            global_data.self_stage = 3U;
        }
    }

    else if (global_data.save_stage == 3U && global_data.self_stage == 3U)
    {
        phi0_pd(1.39f);
        if (global_data.middle_data.l_phi0 < 1.85f && global_data.middle_data.l_phi0 > 1.20f
            && global_data.middle_data.r_phi0 < 1.85f && global_data.middle_data.r_phi0 > 1.20f
            && global_data.middle_data.L_l < 0.2f && global_data.middle_data.L_r < 0.2f)
        {
            global_data.save_stage = 1U;
            global_data.self_stage = 1U;
        }
    }
}
void save_pre()
{
    global_data.jump_stage = 0U;
    global_data.climb_stage = 0U;
    global_data.self_stage = 2U;
    global_data.save_stage = 2U;
}