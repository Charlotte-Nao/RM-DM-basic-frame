//
// Created by ASUS on 2026/10/16.
//

#include "lqr.h"
#include "../State/state.h"
#include "../LegLength/leg_length.h"
#include "../VMC/vmc.h"
#include "../../device/motor/motor_internal.h"
#include "../../device/motor/motor_instance.h"
#include "cmsis_os2.h"
#include <math.h>

float K_l[2][6], K_r[2][6];
float T_w_l, T_p_l, T_w_r, T_p_r;
float tao_lf, tao_lb, tao_rf, tao_rb;
float current_l, current_r;

void lqr_calculate(void)
{
    float length[2] = {global_data.middle_data.L_l, global_data.middle_data.L_r};
    float state[6] = {
        global_data.state_data.theta_rad,
        global_data.state_data.theta_dot_rad_s,
        global_data.state_data.x_m,
        global_data.state_data.x_dot_m_s,
        global_data.state_data.phi_rad,
        global_data.state_data.phi_dot_rad_s
    };

    struct motor_device *wheel_l = motor_get_device("M3508_L");
    struct motor_device *wheel_r = motor_get_device("M3508_R");
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    m3508_data_t *wheel_l_data = wheel_l->motor_data;
    m3508_data_t *wheel_r_data = wheel_r->motor_data;

    T_w_l = T_p_l = T_w_r = T_p_r = 0.0f;
    tao_lf = tao_lb = tao_rf = tao_rb = 0.0f;
    current_l = current_r = 0.0f;
    for (int row = 0; row < 2; row ++ )
    {
        for (int col = 0; col < 6; col ++)
        {
            K_l[row][col] = 0.0f;
            K_r[row][col] = 0.0f;
        }
    }

        // 左右分别按各自腿长求 K。
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

        // J 的转置将各腿 [F,Tp] 转成模型关节力矩
        tao_lf = (global_data.middle_data.Jl[0][0] * F_l + global_data.middle_data.Jl[1][0] * T_p_l) / transmission;
        tao_lb = (global_data.middle_data.Jl[0][1] * F_l + global_data.middle_data.Jl[1][1] * T_p_l) / transmission;
        tao_rf = -(global_data.middle_data.Jr[0][0] * F_r + global_data.middle_data.Jr[1][0] * T_p_r) / transmission;
        tao_rb = -(global_data.middle_data.Jr[0][1] * F_r + global_data.middle_data.Jr[1][1] * T_p_r) / transmission;

        // 左轮反转为建模正方向： 电流码 +/-16384 对应当前驱动的 +/-20 A
        current_l = -T_w_l / (wheel_transmission * wheel_l_data->K_t_Nm_A) * M3508_OUTPUT_LIMIT / M3508_MAX_TORQUE_CURRENT_A;
        current_r = T_w_r / (wheel_transmission * wheel_r_data->K_t_Nm_A) * M3508_OUTPUT_LIMIT / M3508_MAX_TORQUE_CURRENT_A;

        tao_lf = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, tao_lf));
        tao_lb = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, tao_lb));
        tao_rf = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, tao_rf));
        tao_rb = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, tao_rb));
        current_l = fminf(M3508_OUTPUT_LIMIT, fmaxf(-M3508_OUTPUT_LIMIT, current_l));
        current_r = fminf(M3508_OUTPUT_LIMIT, fmaxf(-M3508_OUTPUT_LIMIT, current_r));

    // 急停啊
    if (global_data.remote_data.rc.s1 == 2U)
    {
        for (uint32_t i = 0U; i < Motor_Get_Count(); ++i)
        {
            struct motor_device *motor = motor_instance_get(i);
            if (motor != NULL && motor->send_disable_cmd != NULL)
            {
                motor->send_disable_cmd(motor);
            }
        }
    }
    else
    {
        lf_motor->set_target(lf_motor, 4, 0.0f, 0.0f, 0.0f, tao_lf);
        lb_motor->set_target(lb_motor, 4, 0.0f, 0.0f, 0.0f, tao_lb);
        rf_motor->set_target(rf_motor, 4, 0.0f, 0.0f, 0.0f, tao_rf);
        rb_motor->set_target(rb_motor, 4, 0.0f, 0.0f, 0.0f, tao_rb);
        // 轮端力矩转为电机侧力矩，电流换算及最终限幅由驱动完成。
        wheel_l->set_target(wheel_l, 4, 0.0f, 0.0f, 0.0f, -T_w_l / wheel_transmission);
        wheel_r->set_target(wheel_r, 4, 0.0f, 0.0f, 0.0f, T_w_r / wheel_transmission);
    }
}
