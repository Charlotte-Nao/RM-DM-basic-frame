/** @file pid.c */

#include "pid.h"

static float pid_limit(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void pid_reset(pid_t *pid)
{
    if (pid == 0) {
        return;
    }
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->initialized = 0U;
}

float pid_update(pid_t *pid, float target, float feedback, float dt)
{
    float error;
    float raw_derivative = 0.0f;
    float integral_scale = 1.0f;
    float candidate_integral;
    float candidate_output;
    float output;

    if (pid == 0 || dt <= 0.0f) {
        return 0.0f;
    }

    error = target - feedback;
    if (pid->deadband > 0.0f && error < pid->deadband && error > -pid->deadband) {
        error = 0.0f;
    }
    if (pid->initialized != 0U) {
        raw_derivative = (error - pid->previous_error) / dt;
    } else {
        pid->initialized = 1U;
    }
    pid->filtered_derivative = pid->derivative_filter_alpha * raw_derivative
                             + (1.0f - pid->derivative_filter_alpha)
                             * pid->filtered_derivative;

    /* Integral separation plus variable integration reduce large-error windup. */
    if (pid->integral_separation_threshold > 0.0f &&
        (error > pid->integral_separation_threshold ||
         error < -pid->integral_separation_threshold)) {
        integral_scale = 0.0f;
    } else if (pid->variable_integration_threshold > 0.0f) {
        float abs_error = (error >= 0.0f) ? error : -error;
        float threshold = pid->variable_integration_threshold;
        if (abs_error >= 2.0f * threshold) {
            integral_scale = 0.0f;
        } else if (abs_error > threshold) {
            integral_scale = 2.0f - abs_error / threshold;
        }
    }

    candidate_integral = pid_limit(pid->integral + error * dt * integral_scale,
                                   pid->integral_limit);
    candidate_output = pid->kp * error + pid->ki * candidate_integral
                     + pid->kd * pid->filtered_derivative;
    /* Conditional integration: retain the old integral if it would deepen saturation. */
    if (!((pid->output_limit > 0.0f && candidate_output > pid->output_limit && error * pid->ki > 0.0f) ||
          (pid->output_limit > 0.0f && candidate_output < -pid->output_limit && error * pid->ki < 0.0f))) {
        pid->integral = candidate_integral;
    }
    pid->previous_error = error;

    output = pid->kp * error + pid->ki * pid->integral
           + pid->kd * pid->filtered_derivative;
    return pid_limit(output, pid->output_limit);
}
