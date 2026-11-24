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
#include "../../dsp/LegLength/leg_length.h"
#include <math.h>
void save_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");

    if (uart1 == NULL || lf_motor == NULL || lb_motor == NULL || rf_motor == NULL || rb_motor == NULL) {
        return;
    }

    // 先记录本次数据，再分组发送，避免发送间隔中阶段和控制量持续变化。
    uint32_t tick = osKernelGetTickCount();
    uint8_t s1 = global_data.remote_data.rc.s1;
    uint8_t s2 = global_data.remote_data.rc.s2;
    uint8_t mode = global_data.mode;
    uint8_t mode2 = global_data.mode2;
    uint8_t self_stage = global_data.self_stage;
    uint8_t save_stage = global_data.save_stage;
    uint8_t jump_stage = global_data.jump_stage;
    uint8_t climb_stage = global_data.climb_stage;
    control_data_t control = global_data.control_data;
    float length_l = global_data.middle_data.L_l;
    float length_r = global_data.middle_data.L_r;
    float length_rate_l = global_data.middle_data.L_l_dot;
    float length_rate_r = global_data.middle_data.L_r_dot;
    float phi0_l = global_data.middle_data.l_phi0;
    float phi0_r = global_data.middle_data.r_phi0;
    float theta = global_data.state_data.theta_rad;
    float phi = global_data.state_data.phi_rad;

    uart1->uart_printf(uart1,
        "SAVE tick=%lu s1=%u s2=%u mode=%u mode2=%u self=%u save=%u jump=%u climb=%u\r\n",
        (unsigned long)tick, (unsigned int)s1, (unsigned int)s2,
        (unsigned int)mode, (unsigned int)mode2, (unsigned int)self_stage,
        (unsigned int)save_stage, (unsigned int)jump_stage, (unsigned int)climb_stage);

    osDelay(20U);

    // 腿长为m，腿长速度为m/s。
    uart1->uart_printf(uart1,
        "LEG L_l=%.4f L_l_set=%.4f L_r=%.4f L_r_set=%.4f dL_l=%.4f dL_r=%.4f\r\n",
        length_l, control.L_l, length_r, control.L_r, length_rate_l, length_rate_r);

    osDelay(20U);


    uart1->uart_printf(uart1,
    "ANGLE swing=%u phi0_l_deg=%.3f phi0_r_deg=%.3f set_rad=1.390  theta_deg=%.2f phi_deg=%.2f\r\n",
    (unsigned int)(self_stage == 3U && save_stage == 3U),
    phi0_l * RAD_TO_DEG, phi0_r* RAD_TO_DEG ,  theta * RAD_TO_DEG, phi * RAD_TO_DEG);

    osDelay(20U);

    // F为轴向力N，Tp为虚拟髋力矩Nm，Tw为轮端力矩Nm。
    uart1->uart_printf(uart1,
        "CTRL F_l=%.2f F_r=%.2f Tp_l=%.2f Tp_r=%.2f Tw_l=%.2f Tw_r=%.2f\r\n",
        control.F_l, control.F_r, control.T_p_l, control.T_p_r, control.T_w_l, control.T_w_r);

    osDelay(20U);

}
void climb_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    if (uart1 == NULL) {
        return;
    }

    // 先读取本次数据，再分三次发送，避免发送间隔中各字段持续变化。
    uint32_t tick = osKernelGetTickCount();
    uint8_t stage = global_data.climb_stage;
    uint8_t jump_stage = global_data.jump_stage;
    uint8_t mode = global_data.mode;
    uint8_t mode2 = global_data.mode2;
    int16_t roller = global_data.remote_data.rc.reserved;
    control_data_t control = global_data.control_data;
    float length_l = global_data.middle_data.L_l;
    float length_r = global_data.middle_data.L_r;
    float phi0_l = global_data.middle_data.l_phi0;
    float phi0_r = global_data.middle_data.r_phi0;
    float phi0_rate_l = global_data.middle_data.l_phi0_dot;
    float phi0_rate_r = global_data.middle_data.r_phi0_dot;
    float fn_l = global_data.middle_data.Fn_l;
    float fn_r = global_data.middle_data.Fn_r;
    float theta = global_data.state_data.theta_rad;
    float phi = global_data.state_data.phi_rad;

    // Tw为当前轮端目标力矩，单位Nm。
    uart1->uart_printf(uart1,
        "CLIMB tick=%lu stage=%u jump=%u mode=%u mode2=%u rc=%d Tw_l=%.3f Tw_r=%.3f\r\n",
        (unsigned long)tick, (unsigned int)stage, (unsigned int)jump_stage,
        (unsigned int)mode, (unsigned int)mode2, (int)roller,
        control.T_w_l, control.T_w_r);

    osDelay(20U);

    // 腿长为m，phi0为rad，dphi0为rad/s。
    uart1->uart_printf(uart1,
        "LEG L_l=%.4f L_l_set=%.4f L_r=%.4f L_r_set=%.4f phi0_l_rad=%.3f phi0_r_rad=%.3f dphi0_l=%.3f dphi0_r=%.3f\r\n",
        length_l, control.L_l, length_r, control.L_r,
        phi0_l, phi0_r, phi0_rate_l, phi0_rate_r);

    osDelay(20U);

    // F/Fn为N，Tp为虚拟髋力矩Nm，theta/phi显示为deg。
    uart1->uart_printf(uart1,
        "CTRL F_l=%.2f F_r=%.2f Tp_l=%.2f Tp_r=%.2f Fn_l=%.2f Fn_r=%.2f theta_deg=%.2f phi_deg=%.2f\r\n",
        control.F_l, control.F_r, control.T_p_l, control.T_p_r,
        fn_l, fn_r, theta * RAD_TO_DEG, phi * RAD_TO_DEG);
}
void Fn_test(void)
{
    struct uart_device *uart1 = uart_get_device("uart1_dma");
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    float torque_lf, torque_lb, torque_rf, torque_rb;

    if (uart1 == NULL || lf_motor == NULL || lb_motor == NULL || rf_motor == NULL || rb_motor == NULL) {
        return;
    }

    lf_motor->get_status(lf_motor, "TORQUE", &torque_lf);
    lb_motor->get_status(lb_motor, "TORQUE", &torque_lb);
    rf_motor->get_status(rf_motor, "TORQUE", &torque_rf);
    rb_motor->get_status(rb_motor, "TORQUE", &torque_rb);

    // az为世界系竖直运动加速度，Fn/Fs为N，theta为deg，TAU_fb为电机侧Nm。
    uart1->uart_printf(uart1,
        "FN tick=%lu stage=%u az=%.3f Fn_l=%.2f Fn_r=%.2f\r\n"
        "L_l=%.4f L_r=%.4f theta_l=%.2f theta_r=%.2f Fs_l=%.2f Fs_r=%.2f\r\n"
        "TAU_fb LF=%.3f LB=%.3f RF=%.3f RB=%.3f\r\n",
        (unsigned long)osKernelGetTickCount(),
        (unsigned int)global_data.jump_stage,
        global_data.imu_data.imu_z_acc_m_s2,
        global_data.middle_data.Fn_l, global_data.middle_data.Fn_r,
        global_data.middle_data.L_l, global_data.middle_data.L_r,
        (global_data.middle_data.l_theta - global_data.state_data.phi_rad) * RAD_TO_DEG,
        (global_data.middle_data.r_theta - global_data.state_data.phi_rad) * RAD_TO_DEG,
        global_data.middle_data.F_spring_divid_l, global_data.middle_data.F_spring_divid_r,
        torque_lf, torque_lb, torque_rf, torque_rb);
}
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
        "TORQUE LF=%.3f LB=%.3f RF=%.3f RB=%.3f WL=%.3f WR=%.3f\r\n",
        lf_torque, lb_torque, rf_torque, rb_torque,
        wheel_l_torque, wheel_r_torque);

    osDelay(20);

    uart1->uart_printf(uart1,"L_l=%.4f phi_0_l=%.2f F_l=%.2f T_p_l=%.2f L_r=%.4f  phi0_r= %.2f F_r=%.2f T_p_r=%.2f\r\n",
        global_data.middle_data.L_l,
        global_data.middle_data.l_phi0 * RAD_TO_DEG,
        global_data.control_data.F_l,
        global_data.control_data.T_p_l,
        global_data.middle_data.L_r,
        global_data.middle_data.r_phi0 * RAD_TO_DEG,
        global_data.control_data.F_r,
        global_data.control_data.T_p_r);


    osDelay(20U);

    uart1->uart_printf(uart1,
    "IL_fb=%.3f IR_fb=%.3f RPM_l=%d RPM_r=%d\r\n",
    wheel_l_data->torque_current, wheel_r_data->torque_current,
    (int)wheel_l_data->speed_rpm, (int)wheel_r_data->speed_rpm);
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

    // LF/LB/RF/RB 为驱动输出指令，*_fb 为电机反馈力矩，单位 Nm。
    uart1->uart_printf(uart1,
        "L_l=%.1f L_l_target=%.3f F_l=%.3f LF=%.3f LB=%.3f LF_fb=%.3f LB_fb=%.3f\r\n"
        "RIGHT L_r=%.1f L_r_target=%.3f F_r=%.3f RF=%.3f RB=%.3f RF_fb=%.3f RB_fb=%.3f\r\n",
        global_data.middle_data.L_l * 1000.0f,
        global_data.control_data.L_l * 1000.0f,
        global_data.control_data.F_l,
        lf_data->output_torque_nm, lb_data->output_torque_nm,
        lf_data->torque_nm, lb_data->torque_nm,
        global_data.middle_data.L_r * 1000.0f,
        global_data.control_data.L_r * 1000.0f,
        global_data.control_data.F_r,
        rf_data->output_torque_nm, rb_data->output_torque_nm,
        rf_data->torque_nm, rb_data->torque_nm);
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