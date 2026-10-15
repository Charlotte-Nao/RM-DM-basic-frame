//
// Created by charlotte on 7/21/26.
//

#include "./uart_task.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <math.h>

#include "../../bsp/uart/uart.h"
#include "../../bus/uart/uart_all.h"
#include "../../protocol/protocol.h"
#include"../../application/global_data.h"
#include "../../dsp/VMC/vmc.h"
#include "../../dsp/LegLength/leg_length.h"
#include "../../dsp/LQR/lqr.h"
#include "../../dsp/State/state.h"
#include "../../device/motor/motor_internal.h"
#include "../../dsp/math.h"
#include "../../bsp/LED/LED.h"



void uart_task(void)
{
    struct uart_device *uart1;
    struct uart_device *uart7;
    struct uart_device *uart10;
    struct uart_device *uart5;
    uart1 = uart_get_device("uart1_dma");
    uart7 = uart_get_device("uart7_dma");
    uart10 = uart_get_device("uart10_dma");
    uart5 = uart_get_device("uart5_dma");
    uart1->uart_recv_callback = uart1_recv_callback;
    uart7->uart_recv_callback = uart7_recv_callback;
    uart10->uart_recv_callback = uart10_recv_callback;
    uart5->uart_recv_callback = uart5_recv_callback;

    struct motor_device *wheel_l = motor_get_device("M3508_L");
    struct motor_device *wheel_r = motor_get_device("M3508_R");
    struct motor_device *lf_motor = motor_get_device("DM8009P_LF");
    struct motor_device *lb_motor = motor_get_device("DM_8009P_LB");
    struct motor_device *rf_motor = motor_get_device("DM_8009P_RF");
    struct motor_device *rb_motor = motor_get_device("DM_8009P_RB");
    m3508_data_t *wheel_l_data = wheel_l->motor_data;
    m3508_data_t *wheel_r_data = wheel_r->motor_data;
    dm8009p_data_t *lf_data = lf_motor->motor_data;
    dm8009p_data_t *lb_data = lb_motor->motor_data;
    dm8009p_data_t *rf_data = rf_motor->motor_data;
    dm8009p_data_t *rb_data = rb_motor->motor_data;


    osDelay(2000);

    for (;;)
    {
        uint32_t imu_tick = global_data.imu_data.imu_update_tick;
        uint32_t imu_age = osKernelGetTickCount() - imu_tick;
        uint32_t imu_limit = osKernelGetTickFreq() * MOTOR_OFFLINE_MS / 1000U;
        float length_l = global_data.middle_data.L_l;
        float length_r = global_data.middle_data.L_r;
        bool length_ok = isfinite(length_l) && length_l >= 0.15f && length_l <= 0.35f &&
            isfinite(length_r) && length_r >= 0.15f && length_r <= 0.35f;
        bool state_ok = isfinite(global_data.state_data.theta_rad) && isfinite(global_data.state_data.theta_dot_rad_s) &&
            isfinite(global_data.state_data.x_m) && isfinite(global_data.state_data.x_dot_m_s) &&
            isfinite(global_data.state_data.phi_rad) && isfinite(global_data.state_data.phi_dot_rad_s);
        bool params_ok = isfinite(transmission) && transmission > 0.0f &&
            isfinite(wheel_transmission) && wheel_transmission > 0.0f &&
            isfinite(wheel_l_data->K_t_Nm_A) && wheel_l_data->K_t_Nm_A > 0.0f &&
            isfinite(wheel_r_data->K_t_Nm_A) && wheel_r_data->K_t_Nm_A > 0.0f;

        uart1->uart_printf(uart1,
            "s1=%d imu_ready=%d imu_age=%lu imu_limit=%lu L_l=%.6f L_r=%.6f L_ok=%d F_l=%.3f F_r=%.3f F_ok=%d state_ok=%d params_ok=%d trans=%.4f wheel_trans=%.4f\r\n",
            global_data.remote_data.rc.s1, global_data.imu_data.imu_ready,
            (unsigned long)imu_age, (unsigned long)imu_limit,
            length_l, length_r, length_ok, F_l, F_r, isfinite(F_l) && isfinite(F_r), state_ok, params_ok,
            transmission, wheel_transmission);
        osDelay(100U);

        uart1->uart_printf(uart1,
            "WL_on=%d WL_en=%d WL_err=%d WL_Kt=%.4f WL_calc=%.1f WL_out=%.1f WR_on=%d WR_en=%d WR_err=%d WR_Kt=%.4f WR_calc=%.1f WR_out=%.1f\r\n",
            motor_is_online(wheel_l), wheel_l_data->enabled, wheel_l_data->error,
            wheel_l_data->K_t_Nm_A, current_l, wheel_l_data->output_current_code,
            motor_is_online(wheel_r), wheel_r_data->enabled, wheel_r_data->error,
            wheel_r_data->K_t_Nm_A, current_r, wheel_r_data->output_current_code);
        osDelay(100U);

        uart1->uart_printf(uart1,
            "LF_on=%d LF_req=%d LF_en=%d LF_status=%d LF_out=%.3f LB_on=%d LB_req=%d LB_en=%d LB_status=%d LB_out=%.3f\r\n",
            motor_is_online(lf_motor), lf_data->enable_requested, lf_data->enabled, lf_data->error, lf_data->output_torque_nm,
            motor_is_online(lb_motor), lb_data->enable_requested, lb_data->enabled, lb_data->error, lb_data->output_torque_nm);
        osDelay(100U);

        uart1->uart_printf(uart1,
            "RF_on=%d RF_req=%d RF_en=%d RF_status=%d RF_out=%.3f RB_on=%d RB_req=%d RB_en=%d RB_status=%d RB_out=%.3f\r\n",
            motor_is_online(rf_motor), rf_data->enable_requested, rf_data->enabled, rf_data->error, rf_data->output_torque_nm,
            motor_is_online(rb_motor), rb_data->enable_requested, rb_data->enabled, rb_data->error, rb_data->output_torque_nm);
        osDelay(100U);

        uart1->uart_printf(uart1,
            "Tw_l=%.3f Tp_l=%.3f Tw_r=%.3f Tp_r=%.3f calc_lf=%.3f calc_lb=%.3f calc_rf=%.3f calc_rb=%.3f\r\n",
            T_w_l, T_p_l, T_w_r, T_p_r, tao_lf, tao_lb, tao_rf, tao_rb);
        osDelay(100U);

        uart1->uart_printf(uart1,
            "theta_deg=%.3f theta_dot_rad_s=%.3f x_m=%.4f v_m_s=%.4f phi_deg=%.3f phi_dot_rad_s=%.3f\r\n",
            global_data.state_data.theta_rad * RAD_TO_DEG,
            global_data.state_data.theta_dot_rad_s,
            global_data.state_data.x_m,
            global_data.state_data.x_dot_m_s,
            global_data.state_data.phi_rad * RAD_TO_DEG,
            global_data.state_data.phi_dot_rad_s);
        osDelay(100U);



    }
}
