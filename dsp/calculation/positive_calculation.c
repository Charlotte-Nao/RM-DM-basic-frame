#include "positive_calculation.h"

#include <math.h>
#include <stddef.h>

#include "../math.h"

#define FOUR_AXIS_LENGTH_EPSILON      1.0e-6f

static float four_axis_normalize_degree(float angle_degree)
{
    while (angle_degree > 360.0f) {
        angle_degree -= 360.0f;
    }

    while (angle_degree <= -360.0f) {
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
    const int16_t current_servo_angle[FOUR_AXIS_SERVO_COUNT],
    float current_pose[FOUR_AXIS_TARGET_POSE_SIZE])
{
    float q1;
    float q2;
    float q23;
    float q234;
    float radial_distance;
    unsigned int index;

    for (index = 0U; index < 4; ++index) {
        if (!four_axis_angle_is_valid((float)current_servo_angle[index])) {
            return;
        }
    }

    q1 = (float)current_servo_angle[0] * DEG_TO_RAD;
    q2 = (float)current_servo_angle[1] * DEG_TO_RAD;
    q23 = (float)(current_servo_angle[1] + current_servo_angle[2]) * DEG_TO_RAD;
    q234 = (float)(current_servo_angle[1] + current_servo_angle[2] + current_servo_angle[3]) * DEG_TO_RAD;

    radial_distance = robotic_arm->l_2 * sinf(q2) + robotic_arm->l_3 * sinf(q23) + robotic_arm->l_4_p * sinf(q234) + robotic_arm->l_4_z * cosf(q23);

    current_pose[0] = radial_distance * cosf(q1);
    current_pose[1] = radial_distance * sinf(q1);
    current_pose[2] = robotic_arm->l_1 + robotic_arm->l_2 * cosf(q2) + robotic_arm->l_3 * cosf(q23) + robotic_arm->l_4_p * cosf(q234) - robotic_arm->l_4_z * sinf(q234);
    current_pose[3] = four_axis_normalize_degree(q234 * RAD_TO_DEG);

}
