//
// Created by ASUS on 2026/10/15.
//

#ifndef DM_STATE_H
#define DM_STATE_H

#include "../../application/global_data.h"

extern float wheel_radius;
extern float wheel_transmission;

// VMC 更新腿长后，在新腿长下更新状态矩阵
void state_update(void);

#endif //DM_STATE_H