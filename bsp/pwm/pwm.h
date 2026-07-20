//
// Created by charlotte on 7/20/26.
//

#ifndef DM_PWM_H
#define DM_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum pwm_channel {
    PWM_CHANNEL_1 = 0,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
    PWM_CHANNEL_NUM,
};

void pwm_init(void);
void pwm_start(enum pwm_channel channel);
void pwm_stop(enum pwm_channel channel);
void pwm_set_pulse_us(enum pwm_channel channel, uint16_t pulse_us);
void pwm_power_enable(void);
void pwm_power_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_PWM_H */
