//
// Created by charlotte on 7/20/26.
//

//
// Created by charlotte on 7/20/26.
//

#include "vacuum.h"
#include <stddef.h>
#include <string.h>
#include "../../bsp/pwm/pwm.h"
#include "main.h"

#define VACUUM_SWITCH_OFF_PULSE_US 500U
#define VACUUM_SWITCH_ON_PULSE_US 2500U
#define VACUUM_PUMP_MAX_ON_TIME_MS 60000U
#define VACUUM_VALVE_MAX_ON_TIME_MS 10000U

typedef struct {
    enum pwm_channel pwm_channel;
    uint16_t off_pulse_us;
    uint16_t on_pulse_us;
    uint32_t max_on_time_ms;
    uint32_t on_tick;
    bool enabled;
    bool timed_out;
} vacuum_data_t;

static void vacuum_device_init(struct vacuum_device *device);
static void vacuum_device_enable(struct vacuum_device *device);
static void vacuum_device_disable(struct vacuum_device *device);
static void vacuum_device_update(struct vacuum_device *device);
static void vacuum_device_get_status(const struct vacuum_device *device, const char *which_status, void *status_data);

static vacuum_data_t vacuum_pump_data = {
    .pwm_channel = PWM_CHANNEL_1,
    .off_pulse_us = VACUUM_SWITCH_OFF_PULSE_US,
    .on_pulse_us = VACUUM_SWITCH_ON_PULSE_US,
    .max_on_time_ms = VACUUM_PUMP_MAX_ON_TIME_MS,
};

static struct vacuum_device vacuum_pump = {
    .device_name = "VACUUM_PUMP",
    .device_data = &vacuum_pump_data,
    .init = vacuum_device_init,
    .enable = vacuum_device_enable,
    .disable = vacuum_device_disable,
    .update = vacuum_device_update,
    .get_status = vacuum_device_get_status,
};

static vacuum_data_t vacuum_valve_data = {
    .pwm_channel = PWM_CHANNEL_2,
    .off_pulse_us = VACUUM_SWITCH_OFF_PULSE_US,
    .on_pulse_us = VACUUM_SWITCH_ON_PULSE_US,
    .max_on_time_ms = VACUUM_VALVE_MAX_ON_TIME_MS,
};

static struct vacuum_device vacuum_valve = {
    .device_name = "VACUUM_VALVE",
    .device_data = &vacuum_valve_data,
    .init = vacuum_device_init,
    .enable = vacuum_device_enable,
    .disable = vacuum_device_disable,
    .update = vacuum_device_update,
    .get_status = vacuum_device_get_status,
};

static struct vacuum_device *const vacuum_list[] = {
    &vacuum_pump,
    &vacuum_valve,
};

static void vacuum_device_init(struct vacuum_device *device)
{
    vacuum_data_t *data;

    if (device == NULL || device->device_data == NULL) {
        return;
    }

    data = device->device_data;
    data->on_tick = 0U;
    data->enabled = false;
    data->timed_out = false;
    pwm_set_pulse_us(data->pwm_channel, data->off_pulse_us);
}

static void vacuum_device_enable(struct vacuum_device *device)
{
    vacuum_data_t *data;

    if (device == NULL || device->device_data == NULL) {
        return;
    }

    data = device->device_data;

    if (data->enabled) {
        return;
    }

    pwm_set_pulse_us(data->pwm_channel, data->on_pulse_us);
    data->on_tick = HAL_GetTick();
    data->enabled = true;
    data->timed_out = false;
}

static void vacuum_device_disable(struct vacuum_device *device)
{
    vacuum_data_t *data;

    if (device == NULL || device->device_data == NULL) {
        return;
    }

    data = device->device_data;
    pwm_set_pulse_us(data->pwm_channel, data->off_pulse_us);
    data->on_tick = 0U;
    data->enabled = false;
}

static void vacuum_device_update(struct vacuum_device *device)
{
    vacuum_data_t *data;

    if (device == NULL || device->device_data == NULL) {
        return;
    }

    data = device->device_data;

    if (!data->enabled || data->max_on_time_ms == 0U) {
        return;
    }

    if ((uint32_t)(HAL_GetTick() - data->on_tick) >= data->max_on_time_ms) {
        vacuum_device_disable(device);
        data->timed_out = true;
    }
}

static void vacuum_device_get_status(const struct vacuum_device *device, const char *which_status, void *status_data)
{
    const vacuum_data_t *data;

    if (device == NULL || device->device_data == NULL || which_status == NULL || status_data == NULL) {
        return;
    }

    data = device->device_data;

    if (strcmp(which_status, "STATE") == 0) {
        *(bool *)status_data = data->enabled;
    } else if (strcmp(which_status, "TIMEOUT") == 0) {
        *(bool *)status_data = data->timed_out;
    } else if (strcmp(which_status, "ON_TIME") == 0) {
        *(uint32_t *)status_data = data->enabled ? HAL_GetTick() - data->on_tick : 0U;
    } else if (strcmp(which_status, "MAX_ON_TIME") == 0) {
        *(uint32_t *)status_data = data->max_on_time_ms;
    }
}

struct vacuum_device *vacuum_get_device(const char *name)
{
    uint32_t index;

    if (name == NULL) {
        return NULL;
    }

    for (index = 0U; index < Vacuum_Get_Count(); ++index) {
        if (strcmp(name, vacuum_list[index]->device_name) == 0) {
            return vacuum_list[index];
        }
    }

    return NULL;
}

uint32_t Vacuum_Get_Count(void)
{
    return (uint32_t)(sizeof(vacuum_list) / sizeof(vacuum_list[0]));
}

void Vacuum_System_PowerOn_Init(void)
{
    uint32_t index;

    pwm_init();

    for (index = 0U; index < Vacuum_Get_Count(); ++index) {
        vacuum_list[index]->init(vacuum_list[index]);
    }

    pwm_power_enable();
}

void Vacuum_System_PowerOff(void)
{
    uint32_t index;

    for (index = 0U; index < Vacuum_Get_Count(); ++index) {
        vacuum_list[index]->disable(vacuum_list[index]);
    }

    pwm_power_disable();
}

void Vacuum_All_Update(void)
{
    uint32_t index;

    for (index = 0U; index < Vacuum_Get_Count(); ++index) {
        vacuum_list[index]->update(vacuum_list[index]);
    }
}
