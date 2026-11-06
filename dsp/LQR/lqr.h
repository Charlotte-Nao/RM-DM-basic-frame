//
// Created by ASUS on 2026/10/16.
//

#ifndef DM_LQR_H
#define DM_LQR_H

#include "../../application/global_data.h"

extern float K_l[2][6], K_r[2][6];

// 同 SPR 左右各求 K 和 u=-K*z，目标为零状态，不额外将输出左右除以 2。
void lqr_calculate(void);

#endif //DM_LQR_H