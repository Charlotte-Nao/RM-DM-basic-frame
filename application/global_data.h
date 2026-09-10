//
// Created by charlotte on 7/13/26.
//

#ifndef DM_GLOBAL_DATA_H
#define DM_GLOBAL_DATA_H

#include <stdint.h>

/* Shared IMU attitude.  The sensor task is the sole writer. */
typedef struct {
    volatile float imu_roll_rad;
    volatile float imu_pitch_rad;
    volatile float imu_yaw_rad;
    volatile uint32_t imu_update_tick;
    volatile uint8_t imu_ready;
} imu_data_t;

typedef struct
{
    struct
    {
        volatile int16_t ch0; //右遥杆左右 -660 ~ 660
        volatile int16_t ch1; //右遥杆上下
        volatile int16_t ch2; //左遥杆左右
        volatile int16_t ch3; //左遥杆上下

        volatile uint8_t s1;  //左上拨杆
        volatile uint8_t s2;  //右上拨杆

        volatile uint32_t remote_update_tick;
        volatile uint8_t remote_ready;
    }rc;


} remote_data_t;

typedef struct
{
    imu_data_t imu_data;
    remote_data_t remote_data;
} global_data_t;

extern volatile global_data_t global_data;

#endif //DM_GLOBAL_DATA_H
