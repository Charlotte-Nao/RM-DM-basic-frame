/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_OUT2_EN_Pin GPIO_PIN_13
#define POWER_OUT2_EN_GPIO_Port GPIOC
#define POWER_OUT1_EN_Pin GPIO_PIN_14
#define POWER_OUT1_EN_GPIO_Port GPIOC
#define POWER_5V_EN_Pin GPIO_PIN_15
#define POWER_5V_EN_GPIO_Port GPIOC
#define BMI088_ACC_CS_Pin GPIO_PIN_0
#define BMI088_ACC_CS_GPIO_Port GPIOC
#define BMI088_GYRO_CS_Pin GPIO_PIN_3
#define BMI088_GYRO_CS_GPIO_Port GPIOC
#define BMI088_ACC_INT_Pin GPIO_PIN_10
#define BMI088_ACC_INT_GPIO_Port GPIOE
#define BMI088_GYRO_INT_Pin GPIO_PIN_12
#define BMI088_GYRO_INT_GPIO_Port GPIOE
#define USER_KEY_Pin GPIO_PIN_15
#define USER_KEY_GPIO_Port GPIOA
#define USER_KEY_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
