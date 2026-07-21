#include "global_data.h"

#include "../dsp/calculation/revrse_calculation.h"

volatile global_data_t global_data = {0};
volatile aim_pose_t aim_pose = {0};

struct four_axis_robotic_arm arm = {
    .l_1 = 114.5f,
    .l_2 = 83.7f,
    .l_3 = 120.1f,
    .l_4_p = 50.0f,
    .l_4_z = 89.0f,
};

