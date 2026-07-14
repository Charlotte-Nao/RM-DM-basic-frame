/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../task/all_task.h"
#include "../../device/motor/motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for sensor */
osThreadId_t sensorHandle;
const osThreadAttr_t sensor_attributes = {
  .name = "sensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for printf */
osThreadId_t printfHandle;
const osThreadAttr_t printf_attributes = {
  .name = "printf",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can_test */
osThreadId_t can_testHandle;
const osThreadAttr_t can_test_attributes = {
  .name = "can_test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usb_test */
osThreadId_t usb_testHandle;
const osThreadAttr_t usb_test_attributes = {
  .name = "usb_test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void seneor_task_entry(void *argument);
void printf_entry(void *argument);
void can_test_entry(void *argument);
void usb_test_entry(void *argument);
void test_task_entry(void *argument);
void motor_enter(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensor */
  sensorHandle = osThreadNew(seneor_task_entry, NULL, &sensor_attributes);

  /* creation of printf */
  printfHandle = osThreadNew(printf_entry, NULL, &printf_attributes);

  /* creation of can_test */
  can_testHandle = osThreadNew(can_test_entry, NULL, &can_test_attributes);

  /* creation of usb_test */
  usb_testHandle = osThreadNew(usb_test_entry, NULL, &usb_test_attributes);

  /* creation of test */
  testHandle = osThreadNew(test_task_entry, NULL, &test_attributes);

  /* creation of motor */
  motorHandle = osThreadNew(motor_enter, NULL, &motor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_seneor_task_entry */
/**
  * @brief  Function implementing the sensor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_seneor_task_entry */
void seneor_task_entry(void *argument)
{
  /* USER CODE BEGIN seneor_task_entry */
  sensor_task();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END seneor_task_entry */
}

/* USER CODE BEGIN Header_printf_entry */
/**
* @brief Function implementing the printf thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printf_entry */
void printf_entry(void *argument)
{
  /* USER CODE BEGIN printf_entry */
  printf_task();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END printf_entry */
}

/* USER CODE BEGIN Header_can_test_entry */
/**
* @brief Function implementing the can_test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_test_entry */
void can_test_entry(void *argument)
{
  /* USER CODE BEGIN can_test_entry */
  /* Infinite loop */
  for(;;)
  {
    /* Motor control is owned by test_task during the GM6020 test. */
    osDelay(1000);
  }
  /* USER CODE END can_test_entry */
}

/* USER CODE BEGIN Header_usb_test_entry */
/**
* @brief Function implementing the usb_test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usb_test_entry */
void usb_test_entry(void *argument)
{
  /* USER CODE BEGIN usb_test_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usb_test_entry */
}

/* USER CODE BEGIN Header_test_task_entry */
/**
* @brief Function implementing the test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_test_task_entry */
void test_task_entry(void *argument)
{
  /* USER CODE BEGIN test_task_entry */
  test_task();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task_entry */
}

/* USER CODE BEGIN Header_motor_enter */
/**
* @brief Function implementing the motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_enter */
void motor_enter(void *argument)
{
  /* USER CODE BEGIN motor_enter */
  motor_task();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_enter */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

