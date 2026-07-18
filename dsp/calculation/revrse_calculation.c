/** @file calculation.c */

#include "revrse_calculation.h"

#include <math.h>
#include <stddef.h>

#include "../math.h"
#define FOUR_AXIS_PI                  3.14159265358979323846f
#define FOUR_AXIS_DEG_TO_RAD          (FOUR_AXIS_PI / 180.0f)
#define FOUR_AXIS_RAD_TO_DEG          (180.0f / FOUR_AXIS_PI)

#define FOUR_AXIS_LENGTH_EPSILON      1.0e-6f
#define FOUR_AXIS_REACH_EPSILON       1.0e-5f
#define FOUR_AXIS_DEFAULT_ELBOW_SIGN  1.0f

static float four_axis_clamp(float value, float minimum, float maximum)
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

static int four_axis_solve_one_configuration(float q1_rad,
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

    sine_q3_squared = 1.0f - cosine_q3 * cosine_q3;

    sine_q3_squared = four_axis_clamp(sine_q3_squared, 0.0f, 1.0f);

    sine_q3 = elbow_sign * sqrtf(sine_q3_squared);

    q3_rad = atan2f(sine_q3, cosine_q3);

    q2_rad = atan2f(wrist_height, wrist_radius)
           - atan2f(forearm_length * sine_q3,
                    upper_arm_length + forearm_length * cosine_q3);

    q4_rad = target_pitch_rad - q2_rad - q3_rad;

    candidate[0] = four_axis_normalize_degree(q1_rad * FOUR_AXIS_RAD_TO_DEG);
    candidate[1] = four_axis_normalize_degree(q2_rad * FOUR_AXIS_RAD_TO_DEG);
    candidate[2] = four_axis_normalize_degree(q3_rad * FOUR_AXIS_RAD_TO_DEG);
    candidate[3] = four_axis_normalize_degree(q4_rad * FOUR_AXIS_RAD_TO_DEG);

    for (index = 0U; index < FOUR_AXIS_SERVO_COUNT; ++index) {
        if (!four_axis_angle_is_valid(candidate[index])) {
            return 0;
        }
    }

    for (index = 0U; index < FOUR_AXIS_SERVO_COUNT; ++index) {
        servo_angle[index] = candidate[index];
    }

    return 1;
}

void Four_degree_of_freedom_calculation(const struct four_axis_robotic_arm *robotic_arm,
                                        const float target_pose[FOUR_AXIS_TARGET_POSE_SIZE],
                                        float target_servo_angle[FOUR_AXIS_SERVO_COUNT])
{
    float x;
    float y;
    float z;
    float target_pitch_rad;
    float radial_distance;
    float wrist_radius;
    float wrist_height;
    float effective_forearm_length;
    float fixed_offset_angle;
    float denominator;
    float cosine_effective_q3;
    float sine_effective_q3;
    float q1_rad;
    float q2_rad;
    float q3_rad;
    float q4_rad;
    float candidate[FOUR_AXIS_SERVO_COUNT];
    float elbow_sign;
    unsigned int configuration;
    unsigned int index;

    if (robotic_arm == NULL || target_pose == NULL || target_servo_angle == NULL) {
        return;
    }

    if (!isfinite(robotic_arm->l_1) || !isfinite(robotic_arm->l_2) || !isfinite(robotic_arm->l_3) ||
        !isfinite(robotic_arm->l_4_p) || !isfinite(robotic_arm->l_4_z) || robotic_arm->l_1 < 0.0f ||
        robotic_arm->l_2 <= FOUR_AXIS_LENGTH_EPSILON || robotic_arm->l_3 <= FOUR_AXIS_LENGTH_EPSILON ||
        robotic_arm->l_4_p < 0.0f || robotic_arm->l_4_z < 0.0f) {
        return;
    }

    x = target_pose[0];
    y = target_pose[1];
    z = target_pose[2];

    if (!isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(target_pose[3])) {
        return;
    }

    target_pitch_rad = target_pose[3] * DEG_TO_RAD;
    radial_distance = hypotf(x, y);
    q1_rad = atan2f(y, x);
    wrist_radius = radial_distance - robotic_arm->l_4_p * sinf(target_pitch_rad);
    wrist_height = z - robotic_arm->l_1 - robotic_arm->l_4_p * cosf(target_pitch_rad);
    effective_forearm_length = hypotf(robotic_arm->l_3, robotic_arm->l_4_z);
    fixed_offset_angle = atan2f(robotic_arm->l_4_z, robotic_arm->l_3);
    denominator = 2.0f * robotic_arm->l_2 * effective_forearm_length;
    cosine_effective_q3 = (wrist_radius * wrist_radius + wrist_height * wrist_height -
                           robotic_arm->l_2 * robotic_arm->l_2 - effective_forearm_length * effective_forearm_length) / denominator;

    if (cosine_effective_q3 < (-1.0f - FOUR_AXIS_REACH_EPSILON) || cosine_effective_q3 > (1.0f + FOUR_AXIS_REACH_EPSILON)) {
        return;
    }

    cosine_effective_q3 = four_axis_clamp(cosine_effective_q3, -1.0f, 1.0f);
    for (configuration = 0U; configuration < 2U; ++configuration) {
        elbow_sign = (configuration == 0U) ? FOUR_AXIS_DEFAULT_ELBOW_SIGN : -FOUR_AXIS_DEFAULT_ELBOW_SIGN;
        sine_effective_q3 = elbow_sign * sqrtf(four_axis_clamp(1.0f - cosine_effective_q3 * cosine_effective_q3, 0.0f, 1.0f));
        q3_rad = atan2f(sine_effective_q3, cosine_effective_q3) - fixed_offset_angle;
        q2_rad = atan2f(wrist_radius, wrist_height) - atan2f(effective_forearm_length * sine_effective_q3,
                   robotic_arm->l_2 + effective_forearm_length * cosine_effective_q3);
        q4_rad = target_pitch_rad - q2_rad - q3_rad;

        candidate[0] = four_axis_normalize_degree(q1_rad * RAD_TO_DEG);
        candidate[1] = four_axis_normalize_degree(q2_rad * RAD_TO_DEG);
        candidate[2] = four_axis_normalize_degree(q3_rad * RAD_TO_DEG);
        candidate[3] = four_axis_normalize_degree(q4_rad * RAD_TO_DEG);
        for (index = 0U; index < FOUR_AXIS_SERVO_COUNT; ++index) {
            if (!four_axis_angle_is_valid(candidate[index])) {
                break;
            }
        }
        if (index == FOUR_AXIS_SERVO_COUNT) {
            for (index = 0U; index < FOUR_AXIS_SERVO_COUNT; ++index) {
                target_servo_angle[index] = candidate[index];
            }
            return;
        }
    }
}
