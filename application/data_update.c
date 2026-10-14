//
// Created by ASUS on 2026/10/14.
//

#include "data_update.h"

#include <math.h>
#include <stddef.h>

#include "../dsp/math.h"

static bool config_is_valid(const data_update_config_t *config)
{
    return config != NULL &&
        isfinite(config->upper_leg_length_m) && config->upper_leg_length_m > 0.0f &&
        isfinite(config->lower_leg_length_m) && config->lower_leg_length_m > 0.0f &&
        isfinite(config->wheel_radius_m) && config->wheel_radius_m > 0.0f &&
        (config->x_reference == DATA_UPDATE_X_WHEEL_AXIS || config->x_reference == DATA_UPDATE_X_HIP);
}

static bool motor_mapping_is_valid(const data_update_motor_t *mapping)
{
    return mapping->motor != NULL && mapping->motor->get_status != NULL &&
        (mapping->direction == 1.0f || mapping->direction == -1.0f) &&
        isfinite(mapping->reduction_ratio) && mapping->reduction_ratio > 0.0f &&
        isfinite(mapping->position_zero_rad);
}

/* 调用方在短临界区内读取六个电机，避免 POS/VEL 跨越 CAN 更新。 */
static bool read_motor_feedback(const data_update_motor_t *mapping, bool velocity_is_rpm,
    volatile float *position_rad, volatile float *velocity_rad_s)
{
    float position = NAN;
    float velocity = NAN;

    if (!motor_is_online(mapping->motor))
    {
        return false;
    }

    mapping->motor->get_status(mapping->motor, "POS", &position);
    mapping->motor->get_status(mapping->motor, "VEL", &velocity);

    if (!isfinite(position) || !isfinite(velocity))
    {
        return false;
    }

    if (velocity_is_rpm)
    {
        velocity *= RPM_TO_RAD_S;
    }

    *position_rad = mapping->direction * (position / mapping->reduction_ratio - mapping->position_zero_rad);
    *velocity_rad_s = mapping->direction * velocity / mapping->reduction_ratio;
    return isfinite(*position_rad) && isfinite(*velocity_rad_s);
}

static bool read_leg_feedback(const data_update_leg_motors_t *motors, middle_leg_data_t *leg)
{
    return read_motor_feedback(&motors->phi1, false, &leg->phi1_rad, &leg->phi1_dot_rad_s) &&
        read_motor_feedback(&motors->phi4, false, &leg->phi4_rad, &leg->phi4_dot_rad_s) &&
        read_motor_feedback(&motors->wheel, true, &leg->wheel_position_rad, &leg->wheel_omega_feed_rad_s);
}

/* 共轴简式：L = a*cos(beta) + sqrt(b*b - a*a*sin(beta)^2)。 */
static bool calculate_leg_geometry(middle_leg_data_t *leg, const data_update_config_t *config)
{
    const float a = config->upper_leg_length_m;
    const float b = config->lower_leg_length_m;
    float sin_beta;
    float cos_beta;
    float discriminant;
    float root;
    float k;

    leg->beta_rad = 0.5f * (leg->phi1_rad - leg->phi4_rad);
    leg->phi0_rad = 0.5f * (leg->phi1_rad + leg->phi4_rad);
    leg->phi0_dot_rad_s = 0.5f * (leg->phi1_dot_rad_s + leg->phi4_dot_rad_s);

    if (!isfinite(leg->beta_rad) || !isfinite(leg->phi0_rad) ||
        !isfinite(leg->phi0_dot_rad_s) || fabsf(leg->beta_rad) >= PI)
    {
        return false;
    }

    sin_beta = sinf(leg->beta_rad);
    cos_beta = cosf(leg->beta_rad);
    discriminant = b * b - a * a * sin_beta * sin_beta;
    /* 根号边界不强行钳位：接近退化构型时返回失败。 */
    if (!isfinite(discriminant) || discriminant <= 1.0e-10f)
    {
        return false;
    }

    root = sqrtf(discriminant);
    k = 0.5f * (a * sin_beta + a * a * sin_beta * cos_beta / root);
    leg->length_m = a * cos_beta + root;
    leg->length_dot_m_s = k * (leg->phi4_dot_rad_s - leg->phi1_dot_rad_s);

    return isfinite(leg->length_m) && leg->length_m > 0.0f && isfinite(leg->length_dot_m_s);
}

bool middle_data_update(volatile middle_data_t *middle, const data_update_motors_t *motors,
    const data_update_config_t *config, const volatile imu_data_t *imu, float phi_dot_rad_s)
{
    middle_data_t next = {0};
    uint32_t primask;
    bool feedback_valid;

    if (middle == NULL)
    {
        return false;
    }

    if (motors == NULL || imu == NULL || !config_is_valid(config) || !isfinite(phi_dot_rad_s) ||
        !motor_mapping_is_valid(&motors->left.phi1) || !motor_mapping_is_valid(&motors->left.phi4) || !motor_mapping_is_valid(&motors->left.wheel) ||
        !motor_mapping_is_valid(&motors->right.phi1) || !motor_mapping_is_valid(&motors->right.phi4) || !motor_mapping_is_valid(&motors->right.wheel))
    {
        middle->ready = 0U;
        return false;
    }

    /* 只在读取快照和发布结果时关中断，三角函数不在临界区内运行。
     * 这保证软件缓存读取一致，不代表六个电机物理上同步采样。
     * IMU 时龄由未来任务检查；imu_ready 仅说明已有姿态发布。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    next.phi_rad = imu->imu_pitch_rad;
    next.phi_dot_rad_s = phi_dot_rad_s;
    next.update_tick = imu->imu_update_tick;
    feedback_valid = imu->imu_ready != 0U &&
        read_leg_feedback(&motors->left, &next.left) && read_leg_feedback(&motors->right, &next.right);
    __set_PRIMASK(primask);

    if (!feedback_valid || !isfinite(next.phi_rad) ||
        !calculate_leg_geometry(&next.left, config) || !calculate_leg_geometry(&next.right, config))
    {
        middle->ready = 0U;
        return false;
    }

    next.ready = 1U;
    primask = __get_PRIMASK();
    __disable_irq();
    *middle = next;
    __DMB();
    __set_PRIMASK(primask);
    return true;
}

static bool leg_state_inputs_are_valid(const middle_leg_data_t *leg)
{
    return isfinite(leg->phi0_rad) && isfinite(leg->phi0_dot_rad_s) &&
        isfinite(leg->length_m) && leg->length_m > 0.0f &&
        isfinite(leg->length_dot_m_s) && isfinite(leg->wheel_omega_feed_rad_s);
}

static float leg_horizontal_velocity(const middle_leg_data_t *leg,
    const middle_data_t *middle, const data_update_config_t *config)
{
    const float theta = remainderf(leg->phi0_rad - PI * 0.5f - middle->phi_rad, TWO_PI);
    const float theta_dot = leg->phi0_dot_rad_s - middle->phi_dot_rad_s;

    /* 轮反馈为相对定子角速度；正方向统一后，世界系 Omega = feed + theta_dot。 */
    float velocity = (leg->wheel_omega_feed_rad_s + theta_dot) * config->wheel_radius_m;

    if (config->x_reference == DATA_UPDATE_X_HIP)
    {
        velocity += leg->length_dot_m_s * sinf(theta) + leg->length_m * cosf(theta) * theta_dot;
    }

    return velocity;
}

bool state_data_update(volatile state_data_t *state, const volatile middle_data_t *middle,
    const data_update_config_t *config, float dt_s)
{
    middle_data_t snapshot;
    state_data_t next = {0};
    float previous_x;
    float phi0_mean;
    float left_velocity;
    float right_velocity;
    uint32_t primask;

    if (state == NULL || middle == NULL || !config_is_valid(config) || !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot = *middle;
    previous_x = state->x_m;
    __set_PRIMASK(primask);

    if (snapshot.ready == 0U || !isfinite(previous_x) ||
        !isfinite(snapshot.phi_rad) || !isfinite(snapshot.phi_dot_rad_s) ||
        !leg_state_inputs_are_valid(&snapshot.left) || !leg_state_inputs_are_valid(&snapshot.right))
    {
        return false;
    }

    /* 左右方向角按最短角差求均值，避免 +pi/-pi 的平均值错误。 */
    phi0_mean = snapshot.left.phi0_rad + 0.5f * remainderf(snapshot.right.phi0_rad - snapshot.left.phi0_rad, TWO_PI);
    next.theta_rad = remainderf(phi0_mean - PI * 0.5f - snapshot.phi_rad, TWO_PI);
    next.theta_dot_rad_s = 0.5f * (snapshot.left.phi0_dot_rad_s + snapshot.right.phi0_dot_rad_s) - snapshot.phi_dot_rad_s;
    next.phi_rad = snapshot.phi_rad;
    next.phi_dot_rad_s = snapshot.phi_dot_rad_s;
    left_velocity = leg_horizontal_velocity(&snapshot.left, &snapshot, config);
    right_velocity = leg_horizontal_velocity(&snapshot.right, &snapshot, config);
    next.x_dot_m_s = 0.5f * (left_velocity + right_velocity);
    next.x_m = previous_x + next.x_dot_m_s * dt_s;

    if (!isfinite(next.theta_rad) || !isfinite(next.theta_dot_rad_s) || !isfinite(next.x_m) || !isfinite(next.x_dot_m_s))
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *state = next;
    __DMB();
    __set_PRIMASK(primask);
    return true;
}
