//
// Created by ASUS on 2026/10/14.
//

#ifndef DM_DATA_UPDATE_H
#define DM_DATA_UPDATE_H

#include <stdbool.h>
#include "global_data.h"
#include "../device/motor/motor.h"

/* 标定约定：q = direction * (POS / reduction_ratio - position_zero_rad)。
 * direction 必须为 +1 或 -1；减速比是反馈轴转速 / 模型轴转速。
 * position_zero_rad 使用模型轴单位、方向转换前的 rad。
 * 若 DM 的反馈已经是关节输出轴数据，减速比应为 1，不重复除减速比。
 */
typedef struct
{
    const struct motor_device *motor;
    float direction;
    float reduction_ratio;
    float position_zero_rad;
} data_update_motor_t;

typedef struct
{
    data_update_motor_t phi1;  // DM8009P，POS: rad，VEL: rad/s
    data_update_motor_t phi4;  // DM8009P，POS: rad，VEL: rad/s
    data_update_motor_t wheel; // M3508，POS: rad，VEL: rpm
} data_update_leg_motors_t;

typedef struct
{
    data_update_leg_motors_t left;
    data_update_leg_motors_t right;
} data_update_motors_t;

/* PDF 动力学的 x 为轮轴位移；末页的腿摆修正速度对应髋端平动。
 * 选择髋端口径时，后续 A/B/K 的状态坐标必须同步转换，不能混用。
 */
typedef enum
{
    DATA_UPDATE_X_WHEEL_AXIS = 0,
    DATA_UPDATE_X_HIP = 1
} data_update_x_reference_t;

typedef struct
{
    float upper_leg_length_m; // 实车共轴对称机构：0.21 m
    float lower_leg_length_m; // 实车共轴对称机构：0.25 m
    float wheel_radius_m;     // 实车轮半径，不能使用未标定的默认值
    data_update_x_reference_t x_reference;
} data_update_config_t;

/* 电机反馈 -> 中间变量，不使能电机、不发送指令。
 * 仅支持共轴对称五连杆的正根装配分支；标定后角度必须位于连续工作域。
 * imu_pitch_rad 与 phi_dot_rad_s 必须已转换到 PDF 的机体俯仰方向。
 * 当前 IMU 未发布角速度，由未来任务提供同一采样时刻的 phi_dot_rad_s。
 * 参数配置应在调用期间保持不变；失败保留旧数值，但清除 middle->ready。
 */
bool middle_data_update(volatile middle_data_t *middle, const data_update_motors_t *motors,
    const data_update_config_t *config, const volatile imu_data_t *imu, float phi_dot_rad_s);

/* 中间变量 -> 六状态；dt_s 为两次成功状态更新之间的实际秒数。
 * x_m 从其现有值积分，首次调用前由任务设定初始位置。
 * middle 必须由 middle_data_update 成功生成；失败不改写 state。
 * 不允许运行中切换 x_reference，除非任务同步重置/转换位置状态。
 * 未来任务应先更新 middle，成功后再调用本函数；每帧最多积分一次。
 * 每个输出结构体应只有一个写入任务，多字段读取需取得一致快照。
 */
bool state_data_update(volatile state_data_t *state, const volatile middle_data_t *middle,
    const data_update_config_t *config, float dt_s);

#endif //DM_DATA_UPDATE_H
