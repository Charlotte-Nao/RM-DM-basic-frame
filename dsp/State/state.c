//
// Created by ASUS on 2026/10/15.
//

#include "state.h"
#include "../../device/motor/motor.h"
#include "../math.h"
#include "cmsis_os2.h"
#include <math.h>

float wheel_radius = 0.06f;                  // 轮毂半径
float wheel_transmission = 268.0f / 17.0f;  // 减速比

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
    // if (!global_data.imu_data.imu_ready || !motor_is_online(wheel_l) || !motor_is_online(wheel_r))
    // {
    //     return;
    // }

    //状态矩阵更新
    // phi, phi_dot更新，直接读取BMI088数据
    // 安装结果显示，建模对应 0， 20deg 对应imu数据分别为180deg，160deg， 因而需要转化
    float phi = remainderf(PI - global_data.imu_data.imu_pitch_rad, TWO_PI); // 限幅到-pi 至pi
    float phi_dot = -global_data.imu_data.imu_pitch_dot_rad_s;


    // theta, theta_dot更新
    // phi0 - pi/2 后为腿部坐标系， 减 phi转化为底盘坐标系
    float theta_l = global_data.middle_data.l_theta - phi;
    float theta_r = global_data.middle_data.r_theta - phi;
    float theta_l_dot = global_data.middle_data.l_phi0_dot - phi_dot;
    float theta_r_dot = global_data.middle_data.r_phi0_dot - phi_dot;
    float theta = remainderf((theta_l + theta_r) / 2.0f, TWO_PI);
    float theta_dot = (theta_l_dot + theta_r_dot) / 2.0f;

    float rpm_l = 0.0f;
    float rpm_r = 0.0f;
    wheel_l->get_status(wheel_l, "VEL", &rpm_l);
    wheel_r->get_status(wheel_r, "VEL", &rpm_r);

    // x,x_dot更新
    // 建模x正向移动，左轮反馈为负、右为正；先转轮端，再按 PDF 加上定子转动修正。
    float omega_l = -rpm_l * RPM_TO_RAD_S / wheel_transmission + theta_l_dot;
    float omega_r = rpm_r * RPM_TO_RAD_S / wheel_transmission + theta_r_dot;
    float x_dot = wheel_radius * (omega_l + omega_r) / 2.0f;
    float x = global_data.state_data.x_m + x_dot * dt;

    // 当前 K 的 x 是轮轴位移，暂未补偿转化至底盘坐标系弥补 d(L * sin(theta)) / dt
    global_data.state_data.theta_rad = theta;
    global_data.state_data.theta_dot_rad_s = theta_dot;
    global_data.state_data.x_m = x;
    global_data.state_data.x_dot_m_s = x_dot;
    global_data.state_data.phi_rad = phi;
    global_data.state_data.phi_dot_rad_s = phi_dot;
}
