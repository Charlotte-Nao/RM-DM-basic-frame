/**
 * @file pid.h
 * @brief Reusable position/velocity PID controller.
 */

#ifndef DM_PID_H
#define DM_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float filtered_derivative;
    float integral_limit;
    float output_limit;
    float derivative_filter_alpha;       /* 0: keep previous, 1: no filtering. */
    float deadband;                      /* Error range around zero, <= 0 disables. */
    float integral_separation_threshold; /* Integrate only inside this error, <= 0 disables. */
    float variable_integration_threshold;/* Integration starts reducing above this error. */
    unsigned char initialized;
} pid_t;

void pid_reset(pid_t *pid);

/** Calculate a bounded PID output using a positive time interval in seconds. */
float pid_update(pid_t *pid, float target, float feedback, float dt);

#ifdef __cplusplus
}
#endif

#endif /* DM_PID_H */
