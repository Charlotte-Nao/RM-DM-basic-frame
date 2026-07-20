#ifndef REVRSE_CALCULATION_H
#define REVRSE_CALCULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#define FOUR_AXIS_TARGET_POSE_SIZE         4U
#define FOUR_AXIS_SERVO_COUNT              4U

#define FOUR_AXIS_SERVO_ANGLE_MIN_DEG      (-135.0f)
#define FOUR_AXIS_SERVO_ANGLE_MAX_DEG      (135.0f)

/**
 * @brief 四自由度机械臂结构参数。
 *
 * 所有长度单位必须一致，建议统一使用 mm。
 *
 * l_1:
 *     基座坐标原点到二号关节轴心的竖直高度。
 *
 * l_2:
 *     二号关节轴心到三号关节轴心的距离。
 *
 * l_3:
 *     三号关节轴心到四号关节轴心的距离。
 *
 * l_4_p:
 *     从四号关节轴心沿末端横梁方向，
 *     到吸盘中心轴线的距离。
 *
 * l_4_z:
 *     从末端横梁方向垂直偏移到吸盘接触中心的距离。
 *
 * l_4_p 和 l_4_z 都位于四号关节之后，
 * 因此都随 q2 + q3 + q4 一起转动。
 */
struct four_axis_robotic_arm
{
    float l_1;
    float l_2;
    float l_3;
    float l_4_p;
    float l_4_z;
};

/**
 * @brief 四自由度机械臂逆运动学解算。
 *
 * target_pose:
 *     [0] x，吸盘接触中心的世界坐标 x
 *     [1] y，吸盘接触中心的世界坐标 y
 *     [2] z，吸盘接触中心的世界坐标 z
 *     [3] phi，末端横梁姿态角，单位 degree
 *
 * 当前角度定义：
 *
 *     phi = q2 + q3 + q4
 *
 * 注意：
 * target_pose[3] 表示末端横梁方向，而不是吸盘轴线方向。
 * 如果输入的是吸盘轴线角度，应先根据实际安装方向转换。
 *
 * target_servo_angle:
 *     [0] q1，底座旋转角
 *     [1] q2，肩关节角
 *     [2] q3，肘关节角
 *     [3] q4，腕关节角
 *
 * @return
 *     1：解算成功
 *     0：参数非法、目标不可达或没有满足舵机限位的解
 */
int Four_degree_of_freedom_calculation(
    const struct four_axis_robotic_arm *robotic_arm,
    const float target_pose[FOUR_AXIS_TARGET_POSE_SIZE],
    float target_servo_angle[FOUR_AXIS_SERVO_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* REVRSE_CALCULATION_H */