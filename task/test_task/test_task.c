/**
* @file test_task.c
 * @brief just test
 */

#include "test_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../dsp/math.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/pwm/pwm.h"
#include "../../bsp/usb/usb.h"
#include "../../dsp/VMC/vmc.h"

void test_task(void)
{
    osDelay(1000U);
    // M3508_L使能
    {
        struct motor_device *m3508_l = motor_get_device("M3508_L");
        if (m3508_l != NULL && m3508_l->send_enable_cmd != NULL) {
            m3508_l->send_enable_cmd(m3508_l);
        }
        osDelay(100U);
    }
    // M3508_R使能
    {
        struct motor_device *m3508_r = motor_get_device("M3508_R");
        if (m3508_r != NULL && m3508_r->send_enable_cmd != NULL) {
            m3508_r->send_enable_cmd(m3508_r);
        }
        osDelay(100U);
    }

    // // DM8009P_LF使能

    {
        struct motor_device *dm8009p_lf = motor_get_device("DM8009P_LF");
        if (dm8009p_lf != NULL && dm8009p_lf->send_enable_cmd != NULL) {
            dm8009p_lf->send_enable_cmd(dm8009p_lf);
        }

        osDelay(100U);
    }
    //8009P_LB使能
    {
        struct motor_device *dm_8009p_lb = motor_get_device("DM_8009P_LB");
        if (dm_8009p_lb != NULL && dm_8009p_lb->send_enable_cmd != NULL) {
            dm_8009p_lb->send_enable_cmd(dm_8009p_lb);
        }
        LED_SKY_SET();
        osDelay(100U);
    }
    // DM_8009P_RF使能
     {
         struct motor_device *dm_8009p_rf = motor_get_device("DM_8009P_RF");
         if (dm_8009p_rf != NULL && dm_8009p_rf->send_enable_cmd != NULL) {
             dm_8009p_rf->send_enable_cmd(dm_8009p_rf);
         }
         osDelay(100U);
     }


    // DM_8009P_RB使能
    struct motor_device *dm_8009p_rb = motor_get_device("DM_8009P_RB");
    if (dm_8009p_rb != NULL && dm_8009p_rb->send_enable_cmd != NULL) {
        dm_8009p_rb->send_enable_cmd(dm_8009p_rb);
    }
    osDelay(100U);

    LED_SKY_SET();

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


    for (;;) {
        vmc_calculate();

        osDelay(1);
    }
}
