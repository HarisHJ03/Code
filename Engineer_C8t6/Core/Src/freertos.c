/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* Definitions for Start_Task */
osThreadId_t Start_TaskHandle;
const osThreadAttr_t Start_Task_attributes = {
  .name = "Start_Task",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Msg_Send_Task */
osThreadId_t Msg_Send_TaskHandle;
const osThreadAttr_t Msg_Send_Task_attributes = {
  .name = "Msg_Send_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Detect_Task */
osThreadId_t Detect_TaskHandle;
const osThreadAttr_t Detect_Task_attributes = {
  .name = "Detect_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Info_Get_Task */
osThreadId_t Info_Get_TaskHandle;
const osThreadAttr_t Info_Get_Task_attributes = {
  .name = "Info_Get_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Clamp_Task */
osThreadId_t Clamp_TaskHandle;
const osThreadAttr_t Clamp_Task_attributes = {
  .name = "Clamp_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Task_Fun(void *argument);
void Msg_Send_Task_Fun(void *argument);
void Detect_Task_Fun(void *argument);
void Info_Get_Task_Fun(void *argument);
void Clamp_Task_Fun(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Start_Task */
  Start_TaskHandle = osThreadNew(Start_Task_Fun, NULL, &Start_Task_attributes);

  /* creation of Msg_Send_Task */
  Msg_Send_TaskHandle = osThreadNew(Msg_Send_Task_Fun, NULL, &Msg_Send_Task_attributes);

  /* creation of Detect_Task */
  Detect_TaskHandle = osThreadNew(Detect_Task_Fun, NULL, &Detect_Task_attributes);

  /* creation of Info_Get_Task */
  Info_Get_TaskHandle = osThreadNew(Info_Get_Task_Fun, NULL, &Info_Get_Task_attributes);

  /* creation of Clamp_Task */
  Clamp_TaskHandle = osThreadNew(Clamp_Task_Fun, NULL, &Clamp_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Task_Fun */
/**
  * @brief  Function implementing the Start_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Task_Fun */
//void Start_Task_Fun(void *argument)
//{
//  /* USER CODE BEGIN Start_Task_Fun */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END Start_Task_Fun */
//}

/* USER CODE BEGIN Header_Msg_Send_Task_Fun */
/**
* @brief Function implementing the Msg_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Msg_Send_Task_Fun */
void Msg_Send_Task_Fun(void *argument)
{
  /* USER CODE BEGIN Msg_Send_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Msg_Send_Task_Fun */
}

/* USER CODE BEGIN Header_Detect_Task_Fun */
/**
* @brief Function implementing the Detect_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task_Fun */
void Detect_Task_Fun(void *argument)
{
  /* USER CODE BEGIN Detect_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task_Fun */
}

/* USER CODE BEGIN Header_Info_Get_Task_Fun */
/**
* @brief Function implementing the Info_Get_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Info_Get_Task_Fun */
void Info_Get_Task_Fun(void *argument)
{
  /* USER CODE BEGIN Info_Get_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Info_Get_Task_Fun */
}

/* USER CODE BEGIN Header_Clamp_Task_Fun */
/**
* @brief Function implementing the Clamp_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Clamp_Task_Fun */
void Clamp_Task_Fun(void *argument)
{
  /* USER CODE BEGIN Clamp_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Clamp_Task_Fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

