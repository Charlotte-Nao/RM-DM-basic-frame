//
// Created by ASUS on 2026/10/15.
//

#ifndef DM_LEG_LENGTH_H
#define DM_LEG_LENGTH_H

#include "../../application/global_data.h"

extern float F_l, F_r;
extern float target_length_l, target_length_r;

void leg_pd(void);

#endif //DM_LEG_LENGTH_H
