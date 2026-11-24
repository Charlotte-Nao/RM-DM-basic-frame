//
// Created by charlotte on 7/13/26.
//

#ifndef DM_GLOBAL_DATA_H
#define DM_GLOBAL_DATA_H

#include <stdint.h>

// 遥控器数据
typedef struct
{
    struct
    {
        volatile int16_t ch0; //左水平，中位0，范围-660 ~ 660，
        volatile int16_t ch1; //左竖直，中位0，范围-660 ~ 660，
        volatile int16_t ch2; //右水平，中位0，范围-660 ~ 660，
        volatile int16_t ch3; //右竖直，中位0，范围-660 ~ 660，

        volatile uint8_t s1;  //左拨杆S1：上1、中2、下3
        volatile uint8_t s2;  //右拨杆S2：上1、中2、下3
        volatile int16_t reserved; //滚轮：下-660，上660

        volatile uint32_t remote_update_tick;
        volatile uint8_t remote_ready;
    }rc;
} remote_data_t;

// imu数据
typedef struct {
    volatile float imu_roll_rad;
    volatile float imu_pitch_rad;
    volatile float imu_pitch_dot_rad_s; // 与 imu_pitch 同方向，当前平面运动取去偏 gyro X
    volatile float imu_yaw_rad;
    volatile float imu_yaw_dot_rad_s; // 与 imu_yaw_rad 同方向的偏航角速度
    volatile float imu_z_acc_m_s2; // 世界系竖直运动加速度，向上为正，已去重力
    volatile uint32_t imu_update_tick;
    volatile uint8_t imu_ready;
} imu_data_t;

typedef struct {
    volatile float L_l;
    volatile float L_r;
    volatile float L_l_dot;
    volatile float L_r_dot;
    volatile float l_phi1;
    volatile float l_phi4;
    volatile float l_phi0;
    volatile float l_phi1_dot;
    volatile float l_phi4_dot;
    volatile float l_phi0_dot;
    volatile float r_phi1;
    volatile float r_phi4;
    volatile float r_phi0;
    volatile float r_phi1_dot;
    volatile float r_phi4_dot;
    volatile float r_phi0_dot;
    volatile float l_theta;
    volatile float r_theta;
    volatile float Jl[2][2];
    volatile float Jr[2][2];
    volatile float F_spring_divid_l;
    volatile float F_spring_divid_r;
    volatile float Fn_l;
    volatile float Fn_r;
} middle_data_t;

// 状态空间矩阵：[theta, theta_dot, x, x_dot, phi, phi_dot]。
typedef struct {
    volatile float theta_rad;               // 世界系虚拟腿倾角，rad
    volatile float theta_dot_rad_s;         // 虚拟腿倾角变化率，rad/s
    volatile float x_m;                     // 模型水平位移，m
    volatile float x_dot_m_s;               // 模型水平速度，m/s
    volatile float phi_rad;                 // 机体俯仰角，rad
    volatile float phi_dot_rad_s;           // 机体俯仰角变化率，rad/s
} state_data_t;

typedef struct
{
    volatile float tao_lf;
    volatile float tao_rf;
    volatile float tao_lb;
    volatile float tao_rb;
    volatile float F_l;
    volatile float F_r;
    volatile float T_w_l;
    volatile float T_w_r;
    volatile float T_p_l;
    volatile float T_p_r;
    volatile float L_l;
    volatile float L_r;
} control_data_t;

typedef struct
{
    imu_data_t imu_data;
    remote_data_t remote_data;
    middle_data_t middle_data;
    state_data_t state_data;
    control_data_t control_data;
    uint8_t mode;
    uint8_t mode2;
    uint8_t last_mode;
    uint8_t last_mode2;
    uint8_t jump_stage; // 0U默认，1U准备，2U起跳，3U在空中，4U收腿准备落地，落地变为0U
    uint8_t climb_stage; // 0U默认， 1U升高，2U关LQR，撞击，后摆腿，3U收腿，4U向前回正
    uint8_t save_stage; // 0U未使能， 1U正常状态， 2U收腿. 3U摆腿回正，
    uint8_t self_stage; // 0U异常状态， 1U正常状态, 2U可复位初始姿, 3U复位收腿
    uint8_t last_stage;
} global_data_t;

extern volatile global_data_t global_data;

#endif //DM_GLOBAL_DATA_H
