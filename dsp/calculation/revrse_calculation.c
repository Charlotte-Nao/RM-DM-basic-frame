/**
 * @file revrse_calculation.c
 * @brief Four-axis robotic arm inverse kinematics.
 */

#include "revrse_calculation.h"

#include <math.h>
#include <stddef.h>

#include "../math.h"

/*
 * 长度判定阈值。
 */
#define FOUR_AXIS_LENGTH_EPSILON          1.0e-6f

/*
 * 可达边界浮点误差容限。
 *
 * 余弦理论值应该在 [-1, 1] 范围内，
 * 允许少量浮点计算误差。
 */
#define FOUR_AXIS_REACH_EPSILON           1.0e-5f

/*
 * 当目标位于底座中心轴线上时，
 * q1 理论上不唯一，此时默认令 q1 = 0。
 */
#define FOUR_AXIS_RADIAL_EPSILON          1.0e-6f

/*
 * 默认优先选择的肘部构型。
 *
 *  1.0f：优先选择 q3 为正的构型；
 * -1.0f：优先选择 q3 为负的构型。
 *
 * 如果实际机械臂默认弯曲方向与当前相反，
 * 可以将这里改成 -1.0f。
 */
#define FOUR_AXIS_DEFAULT_ELBOW_SIGN      1.0f

static float four_axis_clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static float four_axis_normalize_degree(float angle_degree)
{
    while (angle_degree > 180.0f) {
        angle_degree -= 360.0f;
    }

    while (angle_degree <= -180.0f) {
        angle_degree += 360.0f;
    }

    return angle_degree;
}

static int four_axis_angle_is_valid(float angle_degree)
{
    return isfinite(angle_degree) &&
           angle_degree >= FOUR_AXIS_SERVO_ANGLE_MIN_DEG &&
           angle_degree <= FOUR_AXIS_SERVO_ANGLE_MAX_DEG;
}

/**
 * @brief 尝试求解一种肘部构型。
 */
static int four_axis_solve_one_configuration(
    float q1_rad,
    float wrist_radius,
    float wrist_height,
    float target_pitch_rad,
    float upper_arm_length,
    float forearm_length,
    float cosine_q3,
    float elbow_sign,
    float servo_angle[FOUR_AXIS_SERVO_COUNT])
{
    float sine_q3_squared;
    float sine_q3;
    float q2_rad;
    float q3_rad;
    float q4_rad;

    float candidate[FOUR_AXIS_SERVO_COUNT];
    unsigned int index;

    /*
     * sin(q3) = ±sqrt(1 - cos²(q3))
     *
     * 正负号对应两种肘部构型。
     */
    sine_q3_squared =
        1.0f - cosine_q3 * cosine_q3;

    sine_q3_squared =
        four_axis_clamp(
            sine_q3_squared,
            0.0f,
            1.0f);

    sine_q3 =
        elbow_sign * sqrtf(sine_q3_squared);

    q3_rad =
        atan2f(sine_q3, cosine_q3);

    /*
     * 当前正运动学中：
     *
     * wrist_radius =
     *     l2 * sin(q2)
     *   + l3 * sin(q2 + q3)
     *
     * wrist_height =
     *     l2 * cos(q2)
     *   + l3 * cos(q2 + q3)
     *
     * 因为关节角是从竖直方向开始计算，
     * 所以这里使用 atan2(radius, height)，
     * 而不是 atan2(height, radius)。
     */
    q2_rad =
        atan2f(wrist_radius, wrist_height) -
        atan2f(
            forearm_length * sine_q3,
            upper_arm_length +
            forearm_length * cosine_q3);

    /*
     * phi = q2 + q3 + q4
     */
    q4_rad =
        target_pitch_rad -
        q2_rad -
        q3_rad;

    candidate[0] =
        four_axis_normalize_degree(
            q1_rad * RAD_TO_DEG);

    candidate[1] =
        four_axis_normalize_degree(
            q2_rad * RAD_TO_DEG);

    candidate[2] =
        four_axis_normalize_degree(
            q3_rad * RAD_TO_DEG);

    candidate[3] =
        four_axis_normalize_degree(
            q4_rad * RAD_TO_DEG);

    /*
     * 检查全部关节是否满足舵机角度范围。
     */
    for (index = 0U;
         index < FOUR_AXIS_SERVO_COUNT;
         ++index) {
        if (!four_axis_angle_is_valid(candidate[index])) {
            return 0;
        }
    }

    /*
     * 只有整个解全部有效时，才写入输出数组。
     */
    for (index = 0U;
         index < FOUR_AXIS_SERVO_COUNT;
         ++index) {
        servo_angle[index] = candidate[index];
    }

    return 1;
}

int Four_degree_of_freedom_calculation(
    const struct four_axis_robotic_arm *robotic_arm,
    const float target_pose[FOUR_AXIS_TARGET_POSE_SIZE],
    float target_servo_angle[FOUR_AXIS_SERVO_COUNT])
{
    float x;
    float y;
    float z;

    float target_pitch_degree;
    float target_pitch_rad;

    float radial_distance;
    float q1_rad;

    float wrist_radius;
    float wrist_height;

    float denominator;
    float cosine_q3;

    float elbow_sign;
    unsigned int configuration;

    /*
     * 指针检查。
     */
    if (robotic_arm == NULL ||
        target_pose == NULL ||
        target_servo_angle == NULL) {
        return 0;
    }

    /*
     * 机械臂参数检查。
     */
    if (!isfinite(robotic_arm->l_1) ||
        !isfinite(robotic_arm->l_2) ||
        !isfinite(robotic_arm->l_3) ||
        !isfinite(robotic_arm->l_4_p) ||
        !isfinite(robotic_arm->l_4_z)) {
        return 0;
    }

    if (robotic_arm->l_1 < 0.0f ||
        robotic_arm->l_2 <= FOUR_AXIS_LENGTH_EPSILON ||
        robotic_arm->l_3 <= FOUR_AXIS_LENGTH_EPSILON ||
        robotic_arm->l_4_p < 0.0f ||
        robotic_arm->l_4_z < 0.0f) {
        return 0;
    }

    x = target_pose[0];
    y = target_pose[1];
    z = target_pose[2];

    target_pitch_degree = target_pose[3];

    if (!isfinite(x) ||
        !isfinite(y) ||
        !isfinite(z) ||
        !isfinite(target_pitch_degree)) {
        return 0;
    }

    /*
     * 将目标姿态角归一化至 (-180, 180]。
     */
    target_pitch_degree =
        four_axis_normalize_degree(
            target_pitch_degree);

    target_pitch_rad =
        target_pitch_degree * DEG_TO_RAD;

    /*
     * 目标点相对底座中心轴线的水平径向距离。
     */
    radial_distance =
        hypotf(x, y);

    /*
     * 底座旋转角。
     *
     * 当目标点位于底座中心轴线上时，
     * q1 没有唯一解，这里默认取 0。
     */
    if (radial_distance <= FOUR_AXIS_RADIAL_EPSILON) {
        q1_rad = 0.0f;
    } else {
        q1_rad = atan2f(y, x);
    }

    /*
     * 当前正运动学末端偏移为：
     *
     * radial_offset =
     *     l_4_p * sin(phi)
     *   + l_4_z * cos(phi)
     *
     * height_offset =
     *     l_4_p * cos(phi)
     *   - l_4_z * sin(phi)
     *
     * 因此必须从目标吸盘位置中同时减去
     * l_4_p 和 l_4_z 的影响。
     *
     * 得到四号关节轴心的位置：
     */
    wrist_radius =
        radial_distance -
        robotic_arm->l_4_p * sinf(target_pitch_rad) -
        robotic_arm->l_4_z * cosf(target_pitch_rad);

    wrist_height =
        z -
        robotic_arm->l_1 -
        robotic_arm->l_4_p * cosf(target_pitch_rad) +
        robotic_arm->l_4_z * sinf(target_pitch_rad);

    /*
     * 对 l2、l3 使用余弦定理：
     *
     * cos(q3) =
     *   (rw² + zw² - l2² - l3²) / (2*l2*l3)
     */
    denominator =
        2.0f *
        robotic_arm->l_2 *
        robotic_arm->l_3;

    if (fabsf(denominator) <= FOUR_AXIS_LENGTH_EPSILON) {
        return 0;
    }

    cosine_q3 =
        (
            wrist_radius * wrist_radius +
            wrist_height * wrist_height -
            robotic_arm->l_2 * robotic_arm->l_2 -
            robotic_arm->l_3 * robotic_arm->l_3
        ) /
        denominator;

    /*
     * 目标超出机械臂可达范围。
     */
    if (cosine_q3 <
            (-1.0f - FOUR_AXIS_REACH_EPSILON) ||
        cosine_q3 >
            (1.0f + FOUR_AXIS_REACH_EPSILON)) {
        return 0;
    }

    /*
     * 消除浮点误差导致的轻微越界。
     */
    cosine_q3 =
        four_axis_clamp(
            cosine_q3,
            -1.0f,
            1.0f);

    /*
     * 尝试两种肘部构型。
     *
     * configuration = 0：
     *     默认构型。
     *
     * configuration = 1：
     *     相反构型。
     */
    for (configuration = 0U;
         configuration < 2U;
         ++configuration) {
        elbow_sign =
            (configuration == 0U)
                ? FOUR_AXIS_DEFAULT_ELBOW_SIGN
                : -FOUR_AXIS_DEFAULT_ELBOW_SIGN;

        if (four_axis_solve_one_configuration(
                q1_rad,
                wrist_radius,
                wrist_height,
                target_pitch_rad,
                robotic_arm->l_2,
                robotic_arm->l_3,
                cosine_q3,
                elbow_sign,
                target_servo_angle)) {
            return 1;
        }
    }

    /*
     * 两种构型都无法满足舵机角度范围。
     */
    return 0;
}