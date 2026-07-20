//
// Created by charlotte on 7/20/26.
//

#include "pwm.h"
#include "main.h"
#include "tim.h"

struct pwm_config {
    TIM_HandleTypeDef *handle;
    uint32_t channel;
};

static const struct pwm_config pwm_configs[PWM_CHANNEL_NUM] = {
    [PWM_CHANNEL_1] = {
        .handle = &htim2,
        .channel = TIM_CHANNEL_1,
    },
    [PWM_CHANNEL_2] = {
        .handle = &htim2,
        .channel = TIM_CHANNEL_3,
    },
    [PWM_CHANNEL_3] = {
        .handle = &htim1,
        .channel = TIM_CHANNEL_1,
    },
    [PWM_CHANNEL_4] = {
        .handle = &htim1,
        .channel = TIM_CHANNEL_3,
    },
};

static const struct pwm_config *pwm_get_config(enum pwm_channel channel)
{
    if ((uint32_t)channel >= PWM_CHANNEL_NUM) {
        return NULL;
    }

    return &pwm_configs[channel];
}

void pwm_init(void)
{
    uint32_t index;

    pwm_power_disable();

    for (index = 0U; index < PWM_CHANNEL_NUM; ++index) {
        __HAL_TIM_SET_COMPARE(pwm_configs[index].handle,
                              pwm_configs[index].channel,
                              0U);

        if (HAL_TIM_PWM_Start(pwm_configs[index].handle,
                              pwm_configs[index].channel) != HAL_OK) {
            Error_Handler();
        }
    }
}

void pwm_start(enum pwm_channel channel)
{
    const struct pwm_config *config = pwm_get_config(channel);

    if (config == NULL) {
        return;
    }

    if (HAL_TIM_PWM_Start(config->handle, config->channel) != HAL_OK) {
        Error_Handler();
    }
}

void pwm_stop(enum pwm_channel channel)
{
    const struct pwm_config *config = pwm_get_config(channel);

    if (config == NULL) {
        return;
    }

    __HAL_TIM_SET_COMPARE(config->handle, config->channel, 0U);

    if (HAL_TIM_PWM_Stop(config->handle, config->channel) != HAL_OK) {
        Error_Handler();
    }
}

void pwm_set_pulse_us(enum pwm_channel channel, uint16_t pulse_us)
{
    const struct pwm_config *config = pwm_get_config(channel);
    uint32_t period;

    if (config == NULL) {
        return;
    }

    period = __HAL_TIM_GET_AUTORELOAD(config->handle) + 1U;

    if ((uint32_t)pulse_us > period) {
        pulse_us = (uint16_t)period;
    }

    __HAL_TIM_SET_COMPARE(config->handle, config->channel, pulse_us);
}

void pwm_power_enable(void)
{
    HAL_GPIO_WritePin(POWER_5V_EN_GPIO_Port,POWER_5V_EN_Pin,GPIO_PIN_SET);
}

void pwm_power_disable(void)
{
    HAL_GPIO_WritePin(POWER_5V_EN_GPIO_Port,POWER_5V_EN_Pin,GPIO_PIN_RESET);
}
