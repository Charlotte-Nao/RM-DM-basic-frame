/**
 * @file motor.h
 * @brief Static, object-like motor device interface.
 */

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

struct motor_device {
    char *motor_name;
    uint32_t motor_id;                 /* Feedback standard CAN identifier. */
    FDCAN_HandleTypeDef *motor_can_handle;
    void *motor_data;
    uint32_t last_rx_tick;

    void (*init)(struct motor_device *motor, uint32_t motor_id,
                 FDCAN_HandleTypeDef *can_handle, int para_num, ...);
    void (*feedback_calculate)(const struct motor_device *motor, const uint8_t data[8]);
    void (*send_enable_cmd)(struct motor_device *motor);
    void (*send_disable_cmd)(struct motor_device *motor);
    void (*send_ctrl_cmd)(struct motor_device *motor);
    void (*update)(struct motor_device *motor);
    void (*set_target)(const struct motor_device *motor, int para_num, ...);
    void (*get_status)(const struct motor_device *motor, const char *which_status,
                       void *status_data);
    void (*set_para)(const struct motor_device *motor, const char *which_para,
                     const void *para_data);
};

struct motor_device *motor_get_device(const char *name);
uint32_t Motor_Get_Count(void);

/** True only after a feedback frame has arrived within the offline timeout. */
bool motor_is_online(const struct motor_device *motor);

/** Initialise the two registered devices in a disabled, zero-output state. */
void Motor_System_PowerOn_Init(void);

/** Call periodically from one control task (nominally 1 kHz). */
void Motor_All_Update(void);

/** Send complete DJI GM6020 0x1FF group frames after all outputs are calculated. */
void Motor_Send_All_Control(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_H */
