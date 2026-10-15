//
// Created by ASUS on 2026/10/15.
//

#ifndef DM_STATE_H
#define DM_STATE_H

#include "../../application/global_data.h"

extern float wheel_radius;
extern float wheel_transmission;

// VMC 更新后连续调用；六状态按当前 MATLAB 的轮轴位移口径更新。
void state_update(void);

#endif //DM_STATE_H