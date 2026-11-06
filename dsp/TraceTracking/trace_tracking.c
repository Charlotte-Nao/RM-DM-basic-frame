//
// Created by ASUS on 2026/11/7.
//

#include "trace_tracking.h"
#include "../LQR/lqr.h"
#include "cmsis_os2.h"

/*
 * 本函数进行遥控器速度跟踪，使用原 K 对六状态误差进行反馈
 * 输出轮力矩与髋力矩，不在此处下发电机控制
 */
void trace_tracking_calculate(void)
{
    static uint32_t last_tick = 0U;
    static float target_x = 0.0f;
    uint32_t now_tick = osKernelGetTickCount();
    float dt = 0.0f;

    if (last_tick != 0U)
    {
        dt = (float)(now_tick - last_tick) / (float)osKernelGetTickFreq();
    }
    else
    {
        // 首次调用时，期望位移与当前实际位移对齐。
        target_x = global_data.state_data.x_m;
    }
    last_tick = now_tick;

    // ch1 的 -660~660 映射到 -1.5~1.5 m/s，正值对应模型前进方向。
    float target_x_dot = global_data.remote_data.rc.ch1 * 1.5f / 660.0f;
    target_x = target_x + target_x_dot * dt;

    // target_x = 0;
    // target_x_dot = 0;

    float T_w_l, T_p_l, T_w_r, T_p_r;
    float length[2] = {global_data.middle_data.L_l, global_data.middle_data.L_r};
    float state[6] = {
        global_data.state_data.theta_rad,
        global_data.state_data.theta_dot_rad_s,
        global_data.state_data.x_m - target_x,
        global_data.state_data.x_dot_m_s - target_x_dot,
        global_data.state_data.phi_rad,
        global_data.state_data.phi_dot_rad_s
    };

    T_w_l = T_p_l = T_w_r = T_p_r = 0.0f;
    for (int row = 0; row < 2; row ++ )
    {
        for (int col = 0; col < 6; col ++)
        {
            K_l[row][col] = 0.0f;
            K_r[row][col] = 0.0f;
        }
    }

    // 左右分别按各自腿长求 K，保持原 LQR 的增益与力矩分配。
    for (int side = 0; side < 2; ++side)
    {
        float L_0 = length[side];
        float t2 = L_0 * L_0;
        float t3 = t2 * L_0;
        float (*gain)[6] = (side == 0) ? K_l : K_r;
        float wheel_torque = 0.0f;
        float hip_torque = 0.0f;

        gain[0][0] = L_0 * (-6.972298677723907e+1f) + t2 * 5.86240998197075e+1f - t3 * 2.056355411564627e+1f - 4.437958705450612f;
        gain[0][1] = L_0 * (-3.613317163281752f) - t2 * 1.586851657043222e+1f + t3 * 1.501969355371675e+1f - 5.61642271527292e-1f;
        gain[0][2] = L_0 * (-7.44955344563899f) + t2 * 7.551282818665943f - t3 * 8.192156164389051e-1f + 8.598221972918148e-2f;
        gain[0][3] = L_0 * (-1.377029511322952e+1f) + t2 * 1.057825076431051e+1f + t3 * 2.331094649043507f - 8.163428948935063e-2f;
        gain[0][4] = L_0 * (-5.297597525520899f) - t2 * 8.554952925629196e+1f + t3 * 1.08948834058635e+2f + 2.391917212177876e+1f;
        gain[0][5] = L_0 * (-1.281900362995862e+1f) + t2 * 2.276024162452228e+1f - t3 * 1.929469466705856e+1f + 4.815765835721892f;

        gain[1][0] = L_0 * 2.807614086056763e+2f - t2 * 8.780623447548282e+2f + t3 * 8.246021328165828e+2f + 4.349654360293358e+1f;
        gain[1][1] = L_0 * 2.59824332537144e+1f - t2 * 3.904525116411644e+1f + t3 * 1.816571753886809e+1f + 8.933163513691218f;
        gain[1][2] = L_0 * 4.346464566652454f - t2 * 6.379543109370046e+1f + t3 * 7.303206859089751e+1f + 1.2337997814599e+1f;
        gain[1][3] = L_0 * 2.51309165651926f - t2 * 1.018841100268989e+2f + t3 * 1.164583127812391e+2f + 2.568481712074224e+1f;
        gain[1][4] = L_0 * 4.51155423693557e+2f - t2 * 5.258199888752926e+2f + t3 * 1.534476916382505e+2f + 1.796827494467812f;
        gain[1][5] = L_0 * 9.204553592315413e+1f - t2 * 1.767187517450367e+2f + t3 * 1.37078008660867e+2f - 8.466501254366102f;

        for (int i = 0; i < 6; ++i)
        {
            wheel_torque = wheel_torque - gain[0][i] * state[i];
            hip_torque = hip_torque - gain[1][i] * state[i];
        }

        wheel_torque = wheel_torque / 2.0f;
        hip_torque = hip_torque / 2.0f;

        if (side == 0)
        {
            T_w_l = wheel_torque;
            T_p_l = hip_torque;
        }
        if (side == 1)
        {
            T_w_r = wheel_torque;
            T_p_r = hip_torque;
        }
    }

    global_data.control_data.T_w_l = T_w_l;
    global_data.control_data.T_w_r = T_w_r;
    global_data.control_data.T_p_l = T_p_l;
    global_data.control_data.T_p_r = T_p_r;
}