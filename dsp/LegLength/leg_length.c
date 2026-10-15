//
// Created by ASUS on 2026/10/15.
//

#include "leg_length.h"
#include "../VMC/vmc.h"
#include "../math.h"
#include "../../device/motor/motor_internal.h"
#include <math.h>

float F_l, F_r;
float F_spring_divid_l, F_spring_divid_r;
float target_length_l, target_length_r;

float kp = 1000.0f;
float kd = 120.0f;
float F_spring = 300.0f;

void leg_pd(void)
{


    // ch1 对应左腿，ch3 对应右腿；-660~660 映射到 0.15~0.4 m。
    target_length_l = 0.15f + (global_data.remote_data.rc.ch1 + 660.0f) * (0.35f - 0.15f) / 1320.0f;
    target_length_r = 0.15f + (global_data.remote_data.rc.ch3 + 660.0f) * (0.35f - 0.15f) / 1320.0f;

    float error_l = target_length_l - global_data.middle_data.L_l;
    float error_r = target_length_r - global_data.middle_data.L_r;

    F_l = kp * error_l - kd * global_data.middle_data.L_l_dot;
    F_r = kp * error_r - kd * global_data.middle_data.L_r_dot;

    // 气弹簧解算，暂时有点问题
    // const float q1 = 0.21f;
    // const float q2 = 0.25f;
    // const float s2 = 0.04863f;
    // const float s3 = 0.20039f;
    // const float alpha = 14.02f * DEG_TO_RAD;
    // float length_l = global_data.middle_data.L_l;
    // float length_r = global_data.middle_data.L_r;
    // float spring_cos_l = (q1 * q1 + q2 * q2 - length_l * length_l) / (2.0f * q1 * q2);
    // float spring_cos_r = (q1 * q1 + q2 * q2 - length_r * length_r) / (2.0f * q1 * q2);
    //
    // F_spring_divid_l = 0.0f;
    // F_spring_divid_r = 0.0f;
    //
    // // PDF 中的夹角是大小腿内角，不是腿摆角 theta；留数值容差避开退化端点。
    // if (length_l > 0.0f && spring_cos_l > -0.999999f && spring_cos_l < 0.999999f)
    // {
    //     float spring_angle_l = acosf(spring_cos_l);
    //     float spring_length_l = sqrtf(s2 * s2 + s3 * s3 - 2.0f * s2 * s3 * cosf(spring_angle_l - alpha));
    //     F_spring_divid_l = F_spring * length_l * s2 * s3 * sinf(spring_angle_l - alpha) /
    //         (spring_length_l * q1 * q2 * sinf(spring_angle_l));
    // }
    //
    // if (length_r > 0.0f && spring_cos_r > -0.999999f && spring_cos_r < 0.999999f)
    // {
    //     float spring_angle_r = acosf(spring_cos_r);
    //     float spring_length_r = sqrtf(s2 * s2 + s3 * s3 - 2.0f * s2 * s3 * cosf(spring_angle_r - alpha));
    //     F_spring_divid_r = F_spring * length_r * s2 * s3 * sinf(spring_angle_r - alpha) /
    //         (spring_length_r * q1 * q2 * sinf(spring_angle_r));
    // }
    //
    // F_l = F_l - F_spring_divid_l;
    // F_r = F_r - F_spring_divid_r;

    // float tao_lf = global_data.middle_data.Jl[0][0] * F_l / transmission;
    // float tao_lb = global_data.middle_data.Jl[0][1] * F_l / transmission;
    // float tao_rf = -global_data.middle_data.Jr[0][0] * F_r / transmission;
    // float tao_rb = -global_data.middle_data.Jr[0][1] * F_r / transmission;
    //
    //
    // dm8009p_data_t *lf_data = motor_get_device("DM8009P_LF")->motor_data;
    // dm8009p_data_t *lb_data = motor_get_device("DM_8009P_LB")->motor_data;
    // dm8009p_data_t *rf_data = motor_get_device("DM_8009P_RF")->motor_data;
    // dm8009p_data_t *rb_data = motor_get_device("DM_8009P_RB")->motor_data;
    //
    // lf_data->output_torque_nm = tao_lf;
    // lb_data->output_torque_nm = tao_lb;
    // rf_data->output_torque_nm = tao_rf;
    // rb_data->output_torque_nm = tao_rb;
}
