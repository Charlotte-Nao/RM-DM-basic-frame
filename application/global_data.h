//
// Created by charlotte on 7/13/26.
//

#ifndef DM_GLOBAL_DATA_H
#define DM_GLOBAL_DATA_H

#include <stdint.h>

struct four_axis_robotic_arm;

/* Shared IMU attitude.  The sensor task is the sole writer. */
typedef struct {
    volatile float imu_roll_rad;
    volatile float imu_pitch_rad;
    volatile float imu_yaw_rad;
    volatile uint32_t imu_update_tick;
    volatile uint8_t imu_ready;
} imu_data_t;



extern volatile imu_data_t imu_data;

#endif //DM_GLOBAL_DATA_H
