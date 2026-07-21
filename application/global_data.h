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
} global_data_t;

typedef struct host_data {
    float x;
    float y;
    float z;
    float phi;
    uint8_t command;
    uint8_t valid;
} aim_pose_t;


extern volatile global_data_t global_data;
extern struct four_axis_robotic_arm arm;
extern  volatile aim_pose_t aim_pose;

#endif //DM_GLOBAL_DATA_H
