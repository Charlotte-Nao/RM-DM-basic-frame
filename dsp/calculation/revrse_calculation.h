#ifndef REVRSE_CALCULATION_H
#define REVRSE_CALCULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#define FOUR_AXIS_TARGET_POSE_SIZE      4U
#define FOUR_AXIS_SERVO_COUNT           4U

#define FOUR_AXIS_SERVO_ANGLE_MIN_DEG  (-135.0f)
#define FOUR_AXIS_SERVO_ANGLE_MAX_DEG   (135.0f)

struct four_axis_robotic_arm
{
    float base_height;
    float upper_arm_length;
    float forearm_length;
    float wrist_length;
};

void Four_degree_of_freedom_calculation(const struct four_axis_robotic_arm *robotic_arm,
                                        const float target_pose[4],
                                        float target_servo_angle[4]);

#ifdef __cplusplus
}
#endif

#endif /* REVRSE_CALCULATION_H */
