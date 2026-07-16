#include "positive_calculation.h"

#include <math.h>
#include <stddef.h>

#define FOUR_AXIS_PI                  3.14159265358979323846f
#define FOUR_AXIS_DEG_TO_RAD          (FOUR_AXIS_PI / 180.0f)
#define FOUR_AXIS_RAD_TO_DEG          (180.0f / FOUR_AXIS_PI)

#define FOUR_AXIS_LENGTH_EPSILON      1.0e-6f

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

void Four_degree_of_freedom_positive_calculation(const struct four_axis_robotic_arm *robotic_arm,
                                                 const float current_servo_angle[FOUR_AXIS_SERVO_COUNT],
                                                 float current_pose[FOUR_AXIS_TARGET_POSE_SIZE])
{
    float q1_rad;
    float q2_rad;
    float q23_rad;
    float q234_rad;

    float radial_distance;
    float pose[FOUR_AXIS_TARGET_POSE_SIZE];

    unsigned int index;

    if (robotic_arm == NULL ||
        current_servo_angle == NULL ||
        current_pose == NULL) {
        return;
    }

    if (!isfinite(robotic_arm->base_height) ||
        !isfinite(robotic_arm->upper_arm_length) ||
        !isfinite(robotic_arm->forearm_length) ||
        !isfinite(robotic_arm->wrist_length) ||
        robotic_arm->base_height < 0.0f ||
        robotic_arm->upper_arm_length <= FOUR_AXIS_LENGTH_EPSILON ||
        robotic_arm->forearm_length <= FOUR_AXIS_LENGTH_EPSILON ||
        robotic_arm->wrist_length < 0.0f) {
        return;
    }

    for (index = 0U; index < FOUR_AXIS_SERVO_COUNT; ++index) {
        if (!four_axis_angle_is_valid(current_servo_angle[index])) {
            return;
        }
    }

    q1_rad = current_servo_angle[0] * FOUR_AXIS_DEG_TO_RAD;
    q2_rad = current_servo_angle[1] * FOUR_AXIS_DEG_TO_RAD;
    q23_rad = (current_servo_angle[1] + current_servo_angle[2]) * FOUR_AXIS_DEG_TO_RAD;
    q234_rad = (current_servo_angle[1] + current_servo_angle[2] + current_servo_angle[3])
             * FOUR_AXIS_DEG_TO_RAD;

    radial_distance = robotic_arm->upper_arm_length * cosf(q2_rad)
                    + robotic_arm->forearm_length * cosf(q23_rad)
                    + robotic_arm->wrist_length * cosf(q234_rad);

    pose[0] = radial_distance * cosf(q1_rad);
    pose[1] = radial_distance * sinf(q1_rad);
    pose[2] = robotic_arm->base_height
            + robotic_arm->upper_arm_length * sinf(q2_rad)
            + robotic_arm->forearm_length * sinf(q23_rad)
            + robotic_arm->wrist_length * sinf(q234_rad);
    pose[3] = four_axis_normalize_degree(q234_rad * FOUR_AXIS_RAD_TO_DEG);

    for (index = 0U; index < FOUR_AXIS_TARGET_POSE_SIZE; ++index) {
        current_pose[index] = pose[index];
    }
}
