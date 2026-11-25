//
// Created by ASUS on 2026/11/25.
//

#include "motor.h"
#include "global_data.h"
#include "../device/motor/motor.h"
#include "../device/motor/motor_internal.h"
#include "../dsp/math.h"
#include <math.h>

void shooter_enable(void)
{
    struct motor_device *left = motor_get_device("M3508_SHOOT_L");
    struct motor_device *right = motor_get_device("M3508_SHOOT_R");

    left->send_enable_cmd(left);
    right->send_enable_cmd(right);
    global_data.gimbal_data.shooter_enabled = 1U;
}
void shooter_disable(void)
{
    struct motor_device *left = motor_get_device("M3508_SHOOT_L");
    struct motor_device *right = motor_get_device("M3508_SHOOT_R");

    global_data.gimbal_data.shooter_enabled = 0U;
    global_data.gimbal_data.shooter_speed_rpm = 0.0f;
    left->send_disable_cmd(left);
    right->send_disable_cmd(right);
}
void pitch_enable(void)
{
    struct motor_device *pitch = motor_get_device("DM4310_PITCH");

    global_data.gimbal_data.pitch_target_rad = global_data.imu_data.imu_pitch_rad;
    pitch->send_enable_cmd(pitch);
    global_data.gimbal_data.pitch_enabled = 1U;
}
void pitch_disable(void)
{
    struct motor_device *pitch = motor_get_device("DM4310_PITCH");

    global_data.gimbal_data.pitch_enabled = 0U;
    pitch->send_disable_cmd(pitch);
}
void shooter_control(void)
{
    if (global_data.gimbal_data.shooter_enabled == 0U)
    {
        return;
    }

    struct motor_device *left = motor_get_device("M3508_SHOOT_L");
    struct motor_device *right = motor_get_device("M3508_SHOOT_R");
    float speed = global_data.gimbal_data.shooter_speed_rpm;

    if (speed > M3508_SPEED_LIMIT_RPM) speed = M3508_SPEED_LIMIT_RPM;
    if (speed < -M3508_SPEED_LIMIT_RPM) speed = -M3508_SPEED_LIMIT_RPM;
    global_data.gimbal_data.shooter_speed_rpm = speed;

    // 摩擦轮位置环关闭，只使用速度环，两只电机反向转动。
    left->set_target(left, 2, 0.0f, speed);
    right->set_target(right, 2, 0.0f, -speed);
}
void pitch_control(void)
{
    if (global_data.gimbal_data.pitch_enabled == 0U)
    {
        return;
    }

    struct motor_device *pitch = motor_get_device("DM4310_PITCH");
    const float pitch_up = 30.0f * DEG_TO_RAD;
    const float pitch_down = -20.0f * DEG_TO_RAD;
    float position = 0.0f;
    float world_pitch = global_data.imu_data.imu_pitch_rad;
    pitch->get_status(pitch, "POS", &position);

    // IMU和编码器均以抬头为正，世界系误差转换为电机位置目标。
    float error = remainderf(global_data.gimbal_data.pitch_target_rad - world_pitch, TWO_PI);
    float target = position + error;
    if (target > pitch_up) target = pitch_up;
    if (target < pitch_down) target = pitch_down;

    global_data.gimbal_data.pitch_motor_target_rad = target;
    // 将世界系目标同步到机械可达位置，避免摇杆在限位外继续累积。
    global_data.gimbal_data.pitch_target_rad = remainderf(world_pitch + target - position, TWO_PI);
    pitch->set_target(pitch, 1, target);
}
