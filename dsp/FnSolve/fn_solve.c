//
// Created by ASUS on 2026/11/15.
//

#include "fn_solve.h"
#include "../LinkageSolution/linkage_solution.h"
#include "../LegLength/leg_length.h"
#include "cmsis_os2.h"
#include <math.h>


void fn_solve()
{
    static uint32_t last_tick = 0U;
    static float last_length_rate[2], last_theta_rate[2];
    static float length_acc[2], theta_acc[2];
    const float wheel_mass = 0.6f;
    const float gravity = 9.8f;
    const float filter_alpha = 0.19f;
    uint32_t now_tick = osKernelGetTickCount();
    float dt = 0.0f;

    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    float torque_lf, torque_lb, torque_rf, torque_rb;
    lf_motor->get_status(lf_motor, "TORQUE", &torque_lf);
    lb_motor->get_status(lb_motor, "TORQUE", &torque_lb);
    rf_motor->get_status(rf_motor, "TORQUE", &torque_rf);
    rb_motor->get_status(rb_motor, "TORQUE", &torque_rb);

    float phi = global_data.state_data.phi_rad;
    float phi_rate = global_data.state_data.phi_dot_rad_s;
    float body_z_acc = global_data.imu_data.imu_z_acc_m_s2;
    float length[2] = {global_data.middle_data.L_l, global_data.middle_data.L_r};
    float length_rate[2] = {global_data.middle_data.L_l_dot, global_data.middle_data.L_r_dot};
    float theta[2] = {global_data.middle_data.l_theta - phi, global_data.middle_data.r_theta - phi};
    float theta_rate[2] = {global_data.middle_data.l_phi0_dot - phi_rate, global_data.middle_data.r_phi0_dot - phi_rate};
    float k[2] = {-global_data.middle_data.Jl[0][0], -global_data.middle_data.Jr[0][0]};
    float spring_force[2] = {F_spring_divid_l, F_spring_divid_r};

    // 反解 torque_send 的映射，力矩反馈为电机侧，右腿方向相反。
    float motor_force[2] = {
        transmission * (torque_lb - torque_lf) / (2.0f * k[0]),
        transmission * (torque_rf - torque_rb) / (2.0f * k[1])
    };
    float hip_torque[2] = {
        transmission * (torque_lf + torque_lb),
        -transmission * (torque_rf + torque_rb)
    };
    float fn[2];

    if (last_tick != 0U)
    {
        dt = (float)(now_tick - last_tick) / (float)osKernelGetTickFreq();
    }

    for (int side = 0; side < 2; ++side)
    {
        // 第一拍只记录速度；后续速度差分采用简单一阶滤波。
        if (last_tick != 0U && dt > 0.0f)
        {
            length_acc[side] = filter_alpha * (length_rate[side] - last_length_rate[side]) / dt
                + (1.0f - filter_alpha) * length_acc[side];
            theta_acc[side] = filter_alpha * (theta_rate[side] - last_theta_rate[side]) / dt
                + (1.0f - filter_alpha) * theta_acc[side];
        }
        last_length_rate[side] = length_rate[side];
        last_theta_rate[side] = theta_rate[side];

        float cos_theta = cosf(theta[side]);
        float sin_theta = sinf(theta[side]);
        float total_force = motor_force[side] + spring_force[side];
        // 当前 VMC 的正 Tp 增大 phi0，切向力向下的投影为 -Tp*sin(theta)/L。
        float p = total_force * cos_theta - hip_torque[side] * sin_theta / length[side];
        float wheel_z_acc = body_z_acc - length_acc[side] * cos_theta
            + 2.0f * length_rate[side] * theta_rate[side] * sin_theta
            + length[side] * theta_acc[side] * sin_theta
            + length[side] * theta_rate[side] * theta_rate[side] * cos_theta;
        fn[side] = p + wheel_mass * (gravity + wheel_z_acc);
    }
    last_tick = now_tick;
    global_data.middle_data.Fn_l = fn[0];
    global_data.middle_data.Fn_r = fn[1];
}