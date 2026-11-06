//
// Created by ASUS on 2026/11/2.
//

#include "comprensive_control.h"
#include "../../application/global_data.h"
#include "../../device/motor/motor.h"
#include "../State/state.h"

void torque_send(void)
{
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    struct motor_device *wheel_l = motor_get_device("M3508_L");
    struct motor_device *wheel_r = motor_get_device("M3508_R");

    // 关节目标已是电机侧力矩，不重复转换方向和传动比。
    lf_motor->set_target(lf_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_lf);
    lb_motor->set_target(lb_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_lb);
    rf_motor->set_target(rf_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_rf);
    rb_motor->set_target(rb_motor, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.tao_rb);

    // 轮端力矩转换为电机侧力矩，左轮反转为建模正方向。
    wheel_l->set_target(wheel_l, 4, 0.0f, 0.0f, 0.0f, -global_data.control_data.T_w_l / wheel_transmission);
    wheel_r->set_target(wheel_r, 4, 0.0f, 0.0f, 0.0f, global_data.control_data.T_w_r / wheel_transmission);
}
