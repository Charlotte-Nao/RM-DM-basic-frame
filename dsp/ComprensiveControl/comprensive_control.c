//
// Created by ASUS on 2026/11/2.
//

#include "comprensive_control.h"
#include "../../application/global_data.h"
#include "../../device/motor/motor.h"
#include "../State/state.h"
#include "../LinkageSolution/linkage_solution.h"

float steering_scale = 0.25f;

void steering_control(void)
{
    float torque = global_data.remote_data.rc.ch0 / 660.0f * 4.0f * steering_scale;

    global_data.control_data.T_w_l += torque / 2.0f;
    global_data.control_data.T_w_r -= torque / 2.0f;
}

void equalize_phi0(void)
{
    const float kp = 20.0f;
    const float kd = 2.0f;

    // 角度差取右减左，角度为 rad，角速度为 rad/s。
    float phi0_error = global_data.middle_data.r_phi0 - global_data.middle_data.l_phi0;
    float phi0_rate_error = global_data.middle_data.r_phi0_dot - global_data.middle_data.l_phi0_dot;
    float torque = -kp * phi0_error - kd * phi0_rate_error;

    global_data.control_data.T_p_l -= torque;
    global_data.control_data.T_p_r += torque;
}

void torque_send(void)
{
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    struct motor_device *wheel_l = motor_get_device("M3508_L");
    struct motor_device *wheel_r = motor_get_device("M3508_R");

    float F_l = global_data.control_data.F_l;
    float F_r = global_data.control_data.F_r;
    float T_p_l = global_data.control_data.T_p_l;
    float T_p_r = global_data.control_data.T_p_r;

    // 雅可比转置将腿轴向力和髋部力矩映射到电机侧，右侧反向。
    global_data.control_data.tao_lf = (global_data.middle_data.Jl[0][0] * F_l + global_data.middle_data.Jl[1][0] * T_p_l) / transmission;
    global_data.control_data.tao_lb = (global_data.middle_data.Jl[0][1] * F_l + global_data.middle_data.Jl[1][1] * T_p_l) / transmission;
    global_data.control_data.tao_rf = -(global_data.middle_data.Jr[0][0] * F_r + global_data.middle_data.Jr[1][0] * T_p_r) / transmission;
    global_data.control_data.tao_rb = -(global_data.middle_data.Jr[0][1] * F_r + global_data.middle_data.Jr[1][1] * T_p_r) / transmission;

    lf_motor->set_target(lf_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_lf);
    lb_motor->set_target(lb_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_lb);
    rf_motor->set_target(rf_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_rf);
    rb_motor->set_target(rb_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_rb);

    // 轮端力矩转换为电机侧力矩，左轮反转为建模正方向。
    wheel_l->set_target(wheel_l, 4, 0.0f, 0.0f, 0.0f, -global_data.control_data.T_w_l / wheel_transmission);
    wheel_r->set_target(wheel_r, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.T_w_r / wheel_transmission);
}

