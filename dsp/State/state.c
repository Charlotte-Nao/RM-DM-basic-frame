//
// Created by ASUS on 2026/10/15.
//

#include "state.h"
#include "../../device/motor/motor.h"
#include "../math.h"
#include "cmsis_os2.h"
#include <math.h>

float wheel_radius = 0.06f;                  // 与当前 lqr_model.m 的 r 一致，单位 m
float wheel_transmission = 268.0f / 17.0f;  // 已确认与 SPR 相同，反馈转子到轮端的总传动比

void state_update(void)
{
    static uint32_t last_tick = 0U;
    uint32_t now_tick = osKernelGetTickCount();
    float dt = 0.0f;

    if (last_tick != 0U)
    {
        dt = (float)(now_tick - last_tick) / (float)osKernelGetTickFreq();
    }
    last_tick = now_tick;

    struct motor_device *wheel_l = motor_get_device("M3508_L");
    struct motor_device *wheel_r = motor_get_device("M3508_R");
    if (!global_data.imu_data.imu_ready || !motor_is_online(wheel_l) || !motor_is_online(wheel_r))
    {
        return;
    }

    float rpm_l = 0.0f;
    float rpm_r = 0.0f;
    wheel_l->get_status(wheel_l, "VEL", &rpm_l);
    wheel_r->get_status(wheel_r, "VEL", &rpm_r);

    // 实测水平时 IMU 为 +/-pi，模型 phi=0；IMU 160 deg 对应模型 +20 deg。
    float phi = remainderf(PI - global_data.imu_data.imu_pitch_rad, TWO_PI);
    float phi_dot = -global_data.imu_data.imu_pitch_dot_rad_s;

    // middle_data 的 theta 已减过 pi/2，此处只补机体俯仰转换。
    float theta_l = global_data.middle_data.l_theta - phi;
    float theta_r = global_data.middle_data.r_theta - phi;
    float theta_l_dot = global_data.middle_data.l_phi0_dot - phi_dot;
    float theta_r_dot = global_data.middle_data.r_phi0_dot - phi_dot;
    float theta = remainderf((theta_l + theta_r) / 2.0f, TWO_PI);
    float theta_dot = (theta_l_dot + theta_r_dot) / 2.0f;

    // 前进时左反馈为负、右为正；先转轮端，再按 PDF 加上定子转动修正。
    float omega_l = -rpm_l * RPM_TO_RAD_S / wheel_transmission + theta_l_dot;
    float omega_r = rpm_r * RPM_TO_RAD_S / wheel_transmission + theta_r_dot;
    float x_dot = wheel_radius * (omega_l + omega_r) / 2.0f;
    float x = global_data.state_data.x_m + x_dot * dt;

    // 当前 K 的 x 是轮轴位移，不添加髋端速度的 L_dot*sin(theta)+L*cos(theta)*theta_dot。
    // x 持续积分，不照搬 SPR 的高速清零策略；首次调用不积分。
    global_data.state_data.theta_rad = theta;
    global_data.state_data.theta_dot_rad_s = theta_dot;
    global_data.state_data.x_m = x;
    global_data.state_data.x_dot_m_s = x_dot;
    global_data.state_data.phi_rad = phi;
    global_data.state_data.phi_dot_rad_s = phi_dot;
}
