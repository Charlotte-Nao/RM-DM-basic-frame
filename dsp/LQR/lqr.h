//
// Created by ASUS on 2026/10/16.
//

#ifndef DM_LQR_H
#define DM_LQR_H

#include "../../application/global_data.h"

extern float K_l[2][6], K_r[2][6];
extern float T_w_l, T_p_l, T_w_r, T_p_r;
extern float tao_lf, tao_lb, tao_rf, tao_rb; // 电机侧力矩，Nm，已限幅
extern float current_l, current_r;         // M3508 CAN 电流码，不是 A，已限幅

// 按 vmc_calculate -> state_update -> leg_pd -> lqr_calculate 的顺序调用。
// 同 SPR 左右各求 K 和 u=-K*z，目标为零状态，不额外将输出左右除以 2。
// 任一腿长超出 0.15~0.35 m、反馈无效或未使能时，六个电机目标清零。
// 本函数会写电机目标，motor_task 随后发送；验证数据和方向后再手动接入调用。
void lqr_calculate(void);

#endif //DM_LQR_H