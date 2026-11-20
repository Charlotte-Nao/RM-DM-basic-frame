//
// Created by ASUS on 2026/11/17.
//

#include "climb.h"
#include "../math.h"
#include <math.h>
void collision_test()
{
    if (global_data.climb_stage == 1U)
    {
        // if (fabsf(global_data.control_data.T_w_l) > 4.0f && fabsf(global_data.control_data.T_w_r) > 4.0f &&
        //     global_data.middle_data.l_phi0 > 2.09f && global_data.middle_data.r_phi0 > 2.09f) // 120deg
        if (fabsf(global_data.control_data.T_w_l) > 5.0f && fabsf(global_data.control_data.T_w_r) > 5.0f ) // 120deg

        {
            global_data.climb_stage = 2U;
        }
    }
}

void phi0_pd(float target_phi0)
{
    const float kp = 14.0f;
    const float kd = 1.0f;
    const float max_out = 40.0f;

    float error_l = remainderf(target_phi0 - global_data.middle_data.l_phi0, TWO_PI);
    float error_r = remainderf(target_phi0 - global_data.middle_data.r_phi0, TWO_PI);
    float torque_l = kp * error_l - kd * global_data.middle_data.l_phi0_dot;
    float torque_r = kp * error_r - kd * global_data.middle_data.r_phi0_dot;

    global_data.control_data.T_p_l = fminf(max_out, fmaxf(-max_out, torque_l));
    global_data.control_data.T_p_r = fminf(max_out, fmaxf(-max_out, torque_r));
    global_data.control_data.T_w_l = 0.0f;
    global_data.control_data.T_w_r = 0.0f;
}

void climb()
{
    collision_test();
    if (global_data.climb_stage == 2U)
    {
        phi0_pd(3.05f);
        if (global_data.middle_data.l_phi0 > 2.87f && global_data.middle_data.r_phi0 > 2.87f) // 165deg
        {
            global_data.climb_stage = 3U;
        }
    }

    if (global_data.climb_stage == 3U)
    {
        phi0_pd(3.05f);
        if (global_data.middle_data.L_l < 0.18f && global_data.middle_data.L_r < 0.18f)
        {
            global_data.climb_stage = 4U;
        }
    }

    if (global_data.climb_stage == 4U)
    {
        phi0_pd(1.39f);
        if (global_data.middle_data.l_phi0 < 1.57f && global_data.middle_data.r_phi0 < 1.57f &&
            global_data.middle_data.L_l < 0.2f && global_data.middle_data.L_r < 0.2f)
        {
            global_data.climb_stage = 0U;
        }
    }
}