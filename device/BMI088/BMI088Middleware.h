#ifndef BMI088_MIDDLEWARE_H
#define BMI088_MIDDLEWARE_H

#include "main.h"
#include <stdint.h>

/*
 * BMI088 使用 SPI2。
 *
 * 加速度计片选：
 * PC0 → CSB1
 *
 * 陀螺仪片选：
 * PC3 → CSB2
 */

#define BMI088_ACCEL_NS_L()                                           \
HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port,                        \
BMI088_ACC_CS_Pin,                              \
GPIO_PIN_RESET)

#define BMI088_ACCEL_NS_H()                                           \
HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port,                        \
BMI088_ACC_CS_Pin,                              \
GPIO_PIN_SET)

#define BMI088_GYRO_NS_L()                                            \
HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port,                       \
BMI088_GYRO_CS_Pin,                             \
GPIO_PIN_RESET)

#define BMI088_GYRO_NS_H()                                            \
HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port,                       \
BMI088_GYRO_CS_Pin,                             \
GPIO_PIN_SET)

void BMI088_GPIO_init(void);
void BMI088_com_init(void);

void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);

uint8_t BMI088_read_write_byte(uint8_t txdata);

#endif