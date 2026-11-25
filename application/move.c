//
// Created by ASUS on 2026/11/25.
//

#include "move.h"
#include "motor.h"
#include "global_data.h"
#include "../dsp/math.h"
#include "cmsis_os2.h"
#include <math.h>

void move(void)
{
    static uint32_t last_tick = 0U;
    const float pitch_speed = 2.5f; // ch3满杆对应的目标角速度，rad/s
    uint32_t now_tick = osKernelGetTickCount();
    float dt = 0.0f;

    if (last_tick != 0U)
    {
        dt = (float)(now_tick - last_tick) / (float)osKernelGetTickFreq();
    }
    last_tick = now_tick;

    if (global_data.gimbal_data.pitch_enabled != 0U)
    {
        // 沿用参考工程方向，ch3为正时降低世界系Pitch目标。
        float pitch_target = global_data.gimbal_data.pitch_target_rad;
        pitch_target -= global_data.gimbal_data.pitch_ch3 / 660.0f * pitch_speed * dt;
        global_data.gimbal_data.pitch_target_rad = remainderf(pitch_target, TWO_PI);
    }

    pitch_control();
    shooter_control();
}