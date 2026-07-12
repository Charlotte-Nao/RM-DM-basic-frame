#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "main.h"

// 定义通讯模式
#define BMI088_USE_SPI

// 片选管脚宏定义
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_GYRO_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0

// 片选控制
#define BMI088_ACCEL_NS_L() HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET)
#define BMI088_ACCEL_NS_H() HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET)
#define BMI088_GYRO_NS_L()  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET)
#define BMI088_GYRO_NS_H()  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET)

// 提供给驱动层的接口
void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);
uint8_t BMI088_read_write_byte(uint8_t txdata);

#endif