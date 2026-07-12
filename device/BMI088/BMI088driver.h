#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "main.h"
#include "BMI088reg.h"

// 灵敏度定义 (根据寄存器配置选择)
#define BMI088_ACCEL_3G_SEN      0.0008974609375f
#define BMI088_GYRO_2000_SEN     0.00106526443603169529841533860381f

typedef float fp32;

// 初始化与读取
uint8_t BMI088_init(void);
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate);

#endif