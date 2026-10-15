//
// Created by ASUS on 2026/10/15.
//

#include "leg_length.h"
#include "../VMC/vmc.h"
#include "../../device/motor/motor_internal.h"

float F_l, F_r;
float target_length_l, target_length_r;

void leg_pd(void)
{
    float kp = 2000.0f;
    float kd = 120.0f;

    // ch1 对应左腿，ch3 对应右腿；-660~660 映射到 0.15~0.4 m。
    target_length_l = 0.15f + (global_data.remote_data.rc.ch1 + 660.0f) * (0.35f - 0.15f) / 1320.0f;
    target_length_r = 0.15f + (global_data.remote_data.rc.ch3 + 660.0f) * (0.35f - 0.15f) / 1320.0f;

    float error_l = target_length_l - global_data.middle_data.L_l;
    float error_r = target_length_r - global_data.middle_data.L_r;

    F_l = kp * error_l - kd * global_data.middle_data.L_l_dot;
    F_r = kp * error_r - kd * global_data.middle_data.L_r_dot;

    float tao_lf = global_data.middle_data.Jl[0][0] * F_l / transmission;
    float tao_lb = global_data.middle_data.Jl[0][1] * F_l / transmission;
    float tao_rf = -global_data.middle_data.Jr[0][0] * F_r / transmission;
    float tao_rb = -global_data.middle_data.Jr[0][1] * F_r / transmission;

    dm8009p_data_t *lf_data = motor_get_device("DM8009P_LF")->motor_data;
    dm8009p_data_t *lb_data = motor_get_device("DM_8009P_LB")->motor_data;
    dm8009p_data_t *rf_data = motor_get_device("DM_8009P_RF")->motor_data;
    dm8009p_data_t *rb_data = motor_get_device("DM_8009P_RB")->motor_data;

    lf_data->output_torque_nm = tao_lf;
    lb_data->output_torque_nm = tao_lb;
    rf_data->output_torque_nm = tao_rf;
    rb_data->output_torque_nm = tao_rb;
}
