#include "global_data.h"

#include "../dsp/calculation/revrse_calculation.h"

volatile global_data_t global_data = {0};

struct four_axis_robotic_arm arm = {
    .l_1 = 107.5f,
    .l_2 = 83.7f,
    .l_3 = 121.5f,
    .l_4_p = 50.0f,
    .l_4_z = 100.0f,
};
