//
// Created by ASUS on 2026/11/11.
//

#include "big_jump.h"

void jump_pre()
{
    global_data.jump_stage = 1U;

}

void jump_start()
{
    if (global_data.mode == 2U && global_data.mode2 == 5U && global_data.last_mode2 != 5U && global_data.jump_stage == 1U)
    {
        global_data.jump_stage = 2U;
    }

}

void jump_air()
{
    static uint8_t air_clock = 0U;
    float contact_force = - 2.0f;
    static float fn[2] = {0};
    fn[0] = global_data.middle_data.Fn_l;
    fn[1] = global_data.middle_data.Fn_r;

    if (global_data.jump_stage == 0U || global_data.jump_stage == 1U)
    {
        air_clock = 0U;
    }

    // 离地进入空中
    if (fn[0] < contact_force && fn[1] < contact_force && global_data.jump_stage == 2U)
    {
        global_data.jump_stage = 3U;
        air_clock = 0U;
    }

    // 离地100ms后开始收腿
    if (global_data.jump_stage == 3U)
    {
        air_clock++;
        if (air_clock >= 100U)
        {
            global_data.jump_stage = 4U;
        }
    }

    // 起跳防抖，落地判定
    if ((global_data.jump_stage == 3U || global_data.jump_stage == 4U)
        && fn[0] > contact_force && fn[1] > contact_force && air_clock > 50U)
    {
        global_data.jump_stage = 0U;
        air_clock = 0U;
    }
}