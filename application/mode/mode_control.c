//
// Created by ASUS on 2026/10/28.
//

#include "mode_control.h"
#include "../global_data.h"
#include "../../device/motor/motor.h"
#include "../../device/motor/motor_instance.h"
#include "../../../bsp/LED/LED.h"
#include "../../dsp/big_jump/big_jump.h"
#include "../../dsp/self_save/self_save.h"


void mode_switch(void)
{
    if (global_data.remote_data.rc.s1 == 1U)
    {
        global_data.mode = 0U;  // 保护模式，全部失能
        LED_RED_SET();
    }
    else if (global_data.remote_data.rc.s1 == 2U)
    {
        global_data.mode = 1U;  // 正常模式，全部使能
        LED_SKY_SET();
    }
    else if (global_data.remote_data.rc.s1 == 3U)
    {
        global_data.mode = 2U;  // 进入大跳模式！
    }

     if (global_data.remote_data.rc.s2 == 1U)
    {
        global_data.mode2 = 3U;  // 底盘继电器强制断电
        LED_PINK_SET();
    }

    else if (global_data.remote_data.rc.s2 == 2U)
    {
        global_data.mode2 = 4U;  // 空闲状态
    }

    else if (global_data.remote_data.rc.s2 == 3U)
    {
        global_data.mode2 = 5U;  // 起跳
    }

    if (global_data.save_stage == 1U && global_data.remote_data.rc.reserved > 600 && global_data.last_stage == 0U)
    {
        global_data.climb_stage = 1U;  // 进入上台阶阶段
    }
    if (global_data.remote_data.rc.reserved < 400 )
    {
        global_data.climb_stage = 0U;
    }

}

void mode_control(void)
{
    mode_switch();

    switch (global_data.mode)
    {
    case 0U:
        send_all_disable();
        global_data.jump_stage = 0U;
        break;
    case 1U:
        send_all_enable();
        global_data.jump_stage = 0U;
        global_data.save_stage = 0U;
        global_data.climb_stage = 0U;
        break;
    case 2U:
        start_jump();
        break;
    default:
        break;
    }

    switch (global_data.mode2)
    {
    case 3U:
        // chassis_disable();   //底盘完全断电
        save_start();
        break;
    case 4U:
        break;
    case 5U:
        jump_start();
        break;
    default:
        break;
    }

    global_data.last_mode = global_data.mode;
    global_data.last_mode2 = global_data.mode2;
    global_data.last_stage = global_data.climb_stage;
}

void send_all_disable(void)
{
    for (uint32_t i = 0U; i < Motor_Get_Count(); ++i)
    {
        struct motor_device *motor = motor_instance_get(i);
        if (motor != NULL && motor->send_disable_cmd != NULL)
        {
            motor->send_disable_cmd(motor);
        }
    }
    global_data.control_data.T_w_l = 0.0f;
    global_data.control_data.T_w_r = 0.0f;
    global_data.control_data.T_p_l = 0.0f;
    global_data.control_data.T_p_r = 0.0f;
}
void send_all_enable(void)
{
    if (global_data.mode == 1U && global_data.last_mode != 1U)
    {
        // 底盘上电
        HAL_GPIO_WritePin(POWER_OUT2_EN_GPIO_Port, POWER_OUT1_EN_Pin, GPIO_PIN_SET);
        // M3508_L使能
        {
            struct motor_device *m3508_l = motor_get_device("M3508_L");
            if (m3508_l != NULL && m3508_l->send_enable_cmd != NULL) {
                m3508_l->send_enable_cmd(m3508_l);
            }
        }
        // M3508_R使能
        {
            struct motor_device *m3508_r = motor_get_device("M3508_R");
            if (m3508_r != NULL && m3508_r->send_enable_cmd != NULL) {
                m3508_r->send_enable_cmd(m3508_r);
            }
        }

        // // DM8009P_LF使能

        {
            struct motor_device *dm8009p_lf = motor_get_device("DM8009P_LF");
            if (dm8009p_lf != NULL && dm8009p_lf->send_enable_cmd != NULL) {
                dm8009p_lf->send_enable_cmd(dm8009p_lf);
            }

        }
        //8009P_LB使能
        {
            struct motor_device *dm_8009p_lb = motor_get_device("DM_8009P_LB");
            if (dm_8009p_lb != NULL && dm_8009p_lb->send_enable_cmd != NULL) {
                dm_8009p_lb->send_enable_cmd(dm_8009p_lb);
            }
        }
        // DM_8009P_RF使能
         {
             struct motor_device *dm_8009p_rf = motor_get_device("DM_8009P_RF");
             if (dm_8009p_rf != NULL && dm_8009p_rf->send_enable_cmd != NULL) {
                 dm_8009p_rf->send_enable_cmd(dm_8009p_rf);
             }
         }


        // DM_8009P_RB使能
        struct motor_device *dm_8009p_rb = motor_get_device("DM_8009P_RB");
        if (dm_8009p_rb != NULL && dm_8009p_rb->send_enable_cmd != NULL) {
            dm_8009p_rb->send_enable_cmd(dm_8009p_rb);
        }

        // // GM6020_YAW使能
        // {
        //     struct motor_device *gm6020_yaw = motor_get_device("GM6020_YAW");
        //     if (gm6020_yaw != NULL && gm6020_yaw->send_enable_cmd != NULL) {
        //         gm6020_yaw->send_enable_cmd(gm6020_yaw);
        //     }
        //     osDelay(100U);
        // }
        // // MG4005E_STIR使能
        // {
        //     struct motor_device *mg4005e_stir = motor_get_device("MG4005E_STIR");
        //     if (mg4005e_stir != NULL && mg4005e_stir->send_enable_cmd != NULL) {
        //         mg4005e_stir->send_enable_cmd(mg4005e_stir);
        //     }
        //     osDelay(100U);
        // }
    }
}
void chassis_disable(void)
{
    HAL_GPIO_WritePin(POWER_OUT2_EN_GPIO_Port, POWER_OUT1_EN_Pin, GPIO_PIN_RESET);
    global_data.jump_stage = 0U;

}
void start_jump(void)
{
    if (global_data.mode == 2U && global_data.last_mode != 2U)
    {
        jump_pre();
    }
}
void save_start(void)
{
    if (global_data.mode2 == 3U && global_data.last_mode2 != 3U)
    {
        if (global_data.save_stage == 0U)
        {
            save_pre();
        }
    }
}