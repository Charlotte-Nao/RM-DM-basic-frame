//
// Created by ASUS on 2026/11/7.
//

#include "uart_printf.h"
#include "cmsis_os2.h"
#include "../../bsp/uart/uart.h"
#include "../../device/motor/motor_internal.h"
#include "../../application/global_data.h"
#include "../../dsp/math.h"
#include "../../dsp/State/state.h"
#include <math.h>

void lqr_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    struct motor_device *wheel_l = motor_get_device("M3508_L");
    struct motor_device *wheel_r = motor_get_device("M3508_R");

    if (uart1 == NULL || wheel_l == NULL || wheel_r == NULL) {
        return;
    }

    m3508_data_t *wheel_l_data = wheel_l->motor_data;
    m3508_data_t *wheel_r_data = wheel_r->motor_data;

    // 仅预览计算结果，不依赖使能和在线状态，也不写入电机目标。
    float lf_torque = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, global_data.control_data.tao_lf));
    float lb_torque = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, global_data.control_data.tao_lb));
    float rf_torque = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, global_data.control_data.tao_rf));
    float rb_torque = fminf(DM8009P_TORQUE_LIMIT_NM, fmaxf(-DM8009P_TORQUE_LIMIT_NM, global_data.control_data.tao_rb));

    // 轮端目标换算为电机侧力矩，按驱动电流上限对应的力矩限幅。
    float wheel_l_limit = M3508_MAX_TORQUE_CURRENT_A * wheel_l_data->K_t_Nm_A;
    float wheel_r_limit = M3508_MAX_TORQUE_CURRENT_A * wheel_r_data->K_t_Nm_A;
    float wheel_l_torque = -global_data.control_data.T_w_l / wheel_transmission;
    float wheel_r_torque = global_data.control_data.T_w_r / wheel_transmission;
    wheel_l_torque = fminf(wheel_l_limit, fmaxf(-wheel_l_limit, wheel_l_torque));
    wheel_r_torque = fminf(wheel_r_limit, fmaxf(-wheel_r_limit, wheel_r_torque));

    uart1->uart_printf(uart1,
        "STATE theta=%.4f theta_dot=%.4f x=%.4f x_dot=%.4f phi=%.4f phi_dot=%.4f\r\n",
        global_data.state_data.theta_rad * RAD_TO_DEG,
        global_data.state_data.theta_dot_rad_s ,
        global_data.state_data.x_m,
        global_data.state_data.x_dot_m_s,
        global_data.state_data.phi_rad * RAD_TO_DEG,
        global_data.state_data.phi_dot_rad_s);

    // 两次 DMA 发送之间留出间隔。
    osDelay(20U);

    uart1->uart_printf(uart1,
        "TORQUE LF=%.3f LB=%.3f RF=%.3f RB=%.3f WL=%.3f WR=%.3f\r\n"
        "L_l=%.4f phi_0_l=%.2f F_l=%.2f T_p_l=%.2f L_r=%.4f  phi0_r= %.2f F_r=%.2f T_p_r=%.2f\r\n",
        lf_torque, lb_torque, rf_torque, rb_torque,
        wheel_l_torque, wheel_r_torque,
        global_data.middle_data.L_l,
        global_data.middle_data.l_phi0 * RAD_TO_DEG,
        global_data.control_data.F_l,
        global_data.control_data.T_p_l,
        global_data.middle_data.L_r,
        global_data.middle_data.r_phi0 * RAD_TO_DEG,
        global_data.control_data.F_r,
        global_data.control_data.T_p_r);
}

void length_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");

    if (uart1 == NULL || lf_motor == NULL || lb_motor == NULL || rf_motor == NULL || rb_motor == NULL) {
        return;
    }

    dm8009p_data_t *lf_data = lf_motor->motor_data;
    dm8009p_data_t *lb_data = lb_motor->motor_data;
    dm8009p_data_t *rf_data = rf_motor->motor_data;
    dm8009p_data_t *rb_data = rb_motor->motor_data;

    // 扭矩为驱动输出指令，不是电机反馈力矩。
    uart1->uart_printf(uart1,
        "L_l=%.1f L_l_target=%.3f F_l=%.3f LF=%.3f LB=%.3f\r\n"
        "RIGHT L_r=%.1f L_r_target=%.3f F_r=%.3f RF=%.3f RB=%.3f\r\n",
        global_data.middle_data.L_l * 1000.0f,
        global_data.control_data.L_l * 1000.0f,
        global_data.control_data.F_l,
        lf_data->output_torque_nm, lb_data->output_torque_nm,
        global_data.middle_data.L_r * 1000.0f,
        global_data.control_data.L_r * 1000.0f,
        global_data.control_data.F_r,
        rf_data->output_torque_nm, rb_data->output_torque_nm);
}

void state_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");

    if (uart1 == NULL) {
        return;
    }

    uart1->uart_printf(uart1,
        "STATE theta_rad=%.4f theta_dot_rad_s=%.4f x_m=%.4f x_dot_m_s=%.4f phi_rad=%.4f phi_dot_rad_s=%.4f\r\n",
        global_data.state_data.theta_rad,
        global_data.state_data.theta_dot_rad_s,
        global_data.state_data.x_m,
        global_data.state_data.x_dot_m_s,
        global_data.state_data.phi_rad,
        global_data.state_data.phi_dot_rad_s);
}

void DM8009P_offset_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    float lf_pos = 0.0f, lb_pos = 0.0f, rf_pos = 0.0f, rb_pos = 0.0f;

    if (uart1 == NULL || lf_motor == NULL || lb_motor == NULL || rf_motor == NULL || rb_motor == NULL) {
        return;
    }

    // POS 为未经连杆方向、安装零偏和传动比转换的电机反馈，单位 rad。
    lf_motor->get_status(lf_motor, "POS", &lf_pos);
    lb_motor->get_status(lb_motor, "POS", &lb_pos);
    rf_motor->get_status(rf_motor, "POS", &rf_pos);
    rb_motor->get_status(rb_motor, "POS", &rb_pos);

    uart1->uart_printf(uart1,
        "LF_pos=%.4f LB_pos=%.4f l_phi1=%.4f l_phi4=%.4f l_phi0=%.4f L_l=%.1f\r\n"
        "RF_pos=%.4f RB_pos=%.4f phi1=%.4f r_phi4=%.4f r_phi0=%.4f L_r=%.1f\r\n",
        lf_pos * RAD_TO_DEG, lb_pos * RAD_TO_DEG,
        global_data.middle_data.l_phi1 * RAD_TO_DEG,
        global_data.middle_data.l_phi4 * RAD_TO_DEG,
        global_data.middle_data.l_phi0 * RAD_TO_DEG,
        global_data.middle_data.L_l * 1000.0f,
        rf_pos * RAD_TO_DEG, rb_pos * RAD_TO_DEG,
        global_data.middle_data.r_phi1 * RAD_TO_DEG,
        global_data.middle_data.r_phi4 * RAD_TO_DEG,
        global_data.middle_data.r_phi0 * RAD_TO_DEG,
        global_data.middle_data.L_r * 1000.0f);
}