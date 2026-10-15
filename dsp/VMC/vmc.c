//
// Created by ASUS on 2026/10/14.
//

#include "vmc.h"
#include "../../dsp/math.h"
#include <math.h>


// 安装零偏，单位 rad
// l_phi1 =+ 11.12deg , l_phi4 -= 63.66deg, r_phi1 +=133.914deg, r_phi4 += 49.518deg 转化为电机零偏
float lf_offset = 303.1675625f * DEG_TO_RAD;
float lb_offset = 225.5121875f * DEG_TO_RAD;
float rf_offset = -133.3859375f * DEG_TO_RAD;
float rb_offset = -86.1044375f * DEG_TO_RAD;

// 传动比
float transmission = 1.1875f;

// 电机原始反馈：pos 为 rad，vel 为 rad/s。
float lf_pos, lb_pos, rf_pos, rb_pos;
float lf_vel, lb_vel, rf_vel, rb_vel;

float l_beta, l_phi0, r_beta, r_phi0;
float l_theta, r_theta;
float L_l, L_r;
float l_beta_dot, r_beta_dot;
float l_phi0_dot, r_phi0_dot;
float L_l_dot, L_r_dot;
float Jl[2][2], JR[2][2];

void vmc_calculate(void)
{
    const float l1 = 0.21f;
    const float l2 = 0.25f;

    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");

    lf_motor->get_status(lf_motor, "POS", &lf_pos);
    lb_motor->get_status(lb_motor, "POS", &lb_pos);
    rf_motor->get_status(rf_motor, "POS", &rf_pos);
    rb_motor->get_status(rb_motor, "POS", &rb_pos);

    lf_motor->get_status(lf_motor, "VEL", &lf_vel);
    lb_motor->get_status(lb_motor, "VEL", &lb_vel);
    rf_motor->get_status(rf_motor, "VEL", &rf_vel);
    rb_motor->get_status(rb_motor, "VEL", &rb_vel);

    // 顺时针为正，车腿前向摆动，角度增加
    lf_pos = - lf_pos;
    lb_pos = - lb_pos;
    rf_pos = - rf_pos;
    rb_pos = - rb_pos;
    lf_vel = - lf_vel;
    lb_vel = - lb_vel;
    rf_vel = - rf_vel;
    rb_vel = - rb_vel;

    lf_pos = lf_pos - lf_offset;
    lb_pos = lb_pos - lb_offset;
    rf_pos = rf_pos - rf_offset;
    rb_pos = rb_pos - rb_offset;

    // 前杆为 phi4，由后电机b控制、前关节为 phi1，由后点击f控制。
    float l_phi1 = - lf_pos / transmission;
    float l_phi4 = - lb_pos / transmission;
    float r_phi1 = rf_pos / transmission;
    float r_phi4 = rb_pos / transmission;

    l_beta =  (l_phi1 - l_phi4) / 2.0f;
    l_phi0 = (l_phi1 + l_phi4) / 2.0f;
    r_beta =  (r_phi1 - r_phi4) / 2.0f;
    r_phi0 = (r_phi1 + r_phi4) / 2.0f;

    l_theta = PI / 2.0f - l_phi0;
    r_theta = PI / 2.0f - r_phi0;

    L_l = l1 * cosf(l_beta) + sqrtf(l2 * l2 - l1 * l1 * sinf(l_beta) * sinf(l_beta));
    L_r = l1 * cosf(r_beta) + sqrtf(l2 * l2 - l1 * l1 * sinf(r_beta) * sinf(r_beta));

    float k_l = 0.5f * (l1 * sinf(l_beta) +
        l1 * l1 * sinf(l_beta) * cosf(l_beta) / sqrtf(l2 * l2 - l1 * l1 * sinf(l_beta) * sinf(l_beta)));
    float k_r = 0.5f * (l1 * sinf(r_beta) +
        l1 * l1 * sinf(r_beta) * cosf(r_beta) / sqrtf(l2 * l2 - l1 * l1 * sinf(r_beta) * sinf(r_beta)));

    // 行对应 L、phi0，列对应 phi1（后关节）、phi4（前关节）。
    Jl[0][0] = -k_l;
    Jl[0][1] = k_l;
    Jl[1][0] = 0.5f;
    Jl[1][1] = 0.5f;

    JR[0][0] = -k_r;
    JR[0][1] = k_r;
    JR[1][0] = 0.5f;
    JR[1][1] = 0.5f;

    // 零偏固定时，模型关节角速度直接取对应电机 VEL。
    float l_phi1_dot = lb_vel;
    float l_phi4_dot = lf_vel;
    float r_phi1_dot = rb_vel;
    float r_phi4_dot = rf_vel;

    l_beta_dot = (l_phi1_dot - l_phi4_dot) / 2.0f;
    r_beta_dot = (r_phi1_dot - r_phi4_dot) / 2.0f;
    L_l_dot = Jl[0][0] * l_phi1_dot + Jl[0][1] * l_phi4_dot;
    L_r_dot = JR[0][0] * r_phi1_dot + JR[0][1] * r_phi4_dot;
    l_phi0_dot = Jl[1][0] * l_phi1_dot + Jl[1][1] * l_phi4_dot;
    r_phi0_dot = JR[1][0] * r_phi1_dot + JR[1][1] * r_phi4_dot;

    global_data.middle_data.L_l = L_l;
    global_data.middle_data.L_r = L_r;
    global_data.middle_data.L_l_dot = L_l_dot;
    global_data.middle_data.L_r_dot = L_r_dot;
    global_data.middle_data.l_phi1 = l_phi1;
    global_data.middle_data.l_phi4 = l_phi4;
    global_data.middle_data.l_phi0 = l_phi0;
    global_data.middle_data.l_phi1_dot = l_phi1_dot;
    global_data.middle_data.l_phi4_dot = l_phi4_dot;
    global_data.middle_data.l_phi0_dot = l_phi0_dot;
    global_data.middle_data.r_phi1 = r_phi1;
    global_data.middle_data.r_phi4 = r_phi4;
    global_data.middle_data.r_phi0 = r_phi0;
    global_data.middle_data.r_phi1_dot = r_phi1_dot;
    global_data.middle_data.r_phi4_dot = r_phi4_dot;
    global_data.middle_data.r_phi0_dot = r_phi0_dot;
    global_data.middle_data.l_theta = l_theta;
    global_data.middle_data.r_theta = r_theta;

    global_data.middle_data.Jl[0][0] = Jl[0][0];
    global_data.middle_data.Jl[0][1] = Jl[0][1];
    global_data.middle_data.Jl[1][0] = Jl[1][0];
    global_data.middle_data.Jl[1][1] = Jl[1][1];
    global_data.middle_data.Jr[0][0] = JR[0][0];
    global_data.middle_data.Jr[0][1] = JR[0][1];
    global_data.middle_data.Jr[1][0] = JR[1][0];
    global_data.middle_data.Jr[1][1] = JR[1][1];
}
