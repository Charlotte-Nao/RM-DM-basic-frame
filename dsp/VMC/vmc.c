//
// Created by ASUS on 2026/10/14.
//

#include "vmc.h"
#include <math.h>

// 安装零偏，单位 rad；由你手动填写，0.0f 仅为占位。
float lf_offset = 0.0f;
float lb_offset = 0.0f;
float rf_offset = 0.0f;
float rb_offset = 0.0f;

// 电机原始反馈：pos 为 rad，vel 为 rad/s。
float lf_pos, lb_pos, rf_pos, rb_pos;
float lf_vel, lb_vel, rf_vel, rb_vel;

float l_beta, l_phi0, r_beta, r_phi0;
float L_l, L_r;
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

    // 此处约定后关节为 phi1、前关节为 phi4；安装正负方向仍需实车确认。
    float l_phi1 = lb_pos - lb_offset;
    float l_phi4 = lf_pos - lf_offset;
    float r_phi1 = rb_pos - rb_offset;
    float r_phi4 = rf_pos - rf_offset;

    l_beta = (l_phi1 - l_phi4) / 2.0f;
    l_phi0 = (l_phi1 + l_phi4) / 2.0f;
    r_beta = (r_phi1 - r_phi4) / 2.0f;
    r_phi0 = (r_phi1 + r_phi4) / 2.0f;

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
}
