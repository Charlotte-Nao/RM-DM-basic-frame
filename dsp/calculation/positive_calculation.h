#ifndef DM_POSITIVE_CALCULATION_H
#define DM_POSITIVE_CALCULATION_H

#include "revrse_calculation.h"

#ifdef __cplusplus
extern "C" {
#endif

void Four_degree_of_freedom_positive_calculation(const struct four_axis_robotic_arm *robotic_arm,
                                                 const float current_servo_angle[FOUR_AXIS_SERVO_COUNT],
                                                 float current_pose[FOUR_AXIS_TARGET_POSE_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* DM_POSITIVE_CALCULATION_H */
