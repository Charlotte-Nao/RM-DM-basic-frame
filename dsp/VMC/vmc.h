//
// Created by ASUS on 2026/10/14.
//

#ifndef DM_VMC_H
#define DM_VMC_H

#include "../../application/global_data.h"
#include "../../device/motor/motor.h"

extern float lf_offset, lb_offset, rf_offset, rb_offset;
extern float lf_pos, lb_pos, rf_pos, rb_pos;
extern float lf_vel, lb_vel, rf_vel, rb_vel;
extern float l_beta, l_phi0, r_beta, r_phi0;
extern float L_l, L_r;
extern float l_beta_dot, r_beta_dot;
extern float l_phi0_dot, r_phi0_dot;
extern float L_l_dot, L_r_dot;
extern float Jl[2][2], JR[2][2];

void vmc_calculate(void);

#endif //DM_VMC_H
