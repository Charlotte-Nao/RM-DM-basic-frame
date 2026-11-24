//
// Created by ASUS on 2026/11/24.
//

#include "gimbal.h"

#include "../../application/global_data.h"
#include "../../device/motor/motor.h"
#include "../../bus/can/can_send.h"
#include "../math.h"
#include "cmsis_os2.h"
#include <math.h>


void gimbal_chase(void)
{
    static float yaw_target = 0.0f;
    static uint32_t last_tick = 0U;
    static uint8_t initialized = 0U;
    const float yaw_speed = 2.5f; // ch2满杆对应的角速度，rad/s
    uint32_t now_tick = osKernelGetTickCount();
    int16_t ch2 = global_data.remote_data.rc.ch2;
    int16_t ch3 = global_data.remote_data.rc.ch3;
    struct motor_device *yaw_motor = motor_get_device("GM6020_YAW");

    gimbal_send(ch3);

    if (global_data.gimbal_data.gimbal_yaw_ready == 0U || global_data.mode == 0U)
    {
        last_tick = now_tick;
        initialized = 0U;
        return;
    }


    float yaw = global_data.gimbal_data.gimbal_yaw_rad;
    if (initialized == 0U)
    {
        yaw_target = yaw;
        last_tick = now_tick;
        initialized = 1U;
    }

    float dt = (float)(now_tick - last_tick) / (float)osKernelGetTickFreq();
    last_tick = now_tick;

    // 沿用参考工程方向，摇杆控制世界系Yaw目标的变化速度。
    yaw_target = remainderf(yaw_target - ch2 / 660.0f * yaw_speed * dt, TWO_PI);
    float yaw_error = remainderf(yaw_target - yaw, TWO_PI);
    float yaw_position = 0.0f;
    yaw_motor->get_status(yaw_motor, "POS", &yaw_position);

    // 世界系角度误差转换为电机位置目标，闭环和发送由motor_task执行。
    yaw_motor->set_target(yaw_motor, 1, yaw_position + yaw_error);
}
void gimbal_select(void)
{
 switch (global_data.gimbal_mode)
 {
  case 0:
    gimbal_chase();
    break;
    default:
  gimbal_chase();
 }
}