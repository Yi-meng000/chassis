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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskUsart */
osThreadId_t TaskUsartHandle;
const osThreadAttr_t TaskUsart_attributes = {
  .name = "TaskUsart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Taskchassis */
osThreadId_t TaskchassisHandle;
const osThreadAttr_t Taskchassis_attributes = {
  .name = "Taskchassis",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Sekirotask */
osThreadId_t SekirotaskHandle;
const osThreadAttr_t Sekirotask_attributes = {
  .name = "Sekirotask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Test */
osThreadId_t TestHandle;
const osThreadAttr_t Test_attributes = {
  .name = "Test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskTraj */
osThreadId_t TaskTrajHandle;
const osThreadAttr_t TaskTraj_attributes = {
  .name = "TaskTraj",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TaskUart(void *argument);
void TaskChassis(void *argument);
void SekiroTask(void *argument);
void Testtask(void *argument);
void TaskTrajctory(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskUsart */
  TaskUsartHandle = osThreadNew(TaskUart, NULL, &TaskUsart_attributes);

  /* creation of Taskchassis */
  TaskchassisHandle = osThreadNew(TaskChassis, NULL, &Taskchassis_attributes);

  /* creation of Sekirotask */
  SekirotaskHandle = osThreadNew(SekiroTask, NULL, &Sekirotask_attributes);

  /* creation of Test */
  TestHandle = osThreadNew(Testtask, NULL, &Test_attributes);

  /* creation of TaskTraj */
  TaskTrajHandle = osThreadNew(TaskTrajctory, NULL, &TaskTraj_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TaskUart */
/**
* @brief Function implementing the TaskUsart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskUart */
__weak void TaskUart(void *argument)
{
  /* USER CODE BEGIN TaskUart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskUart */
}

/* USER CODE BEGIN Header_TaskChassis */
/**
* @brief Function implementing the Taskchassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskChassis */
__weak void TaskChassis(void *argument)
{
  /* USER CODE BEGIN TaskChassis */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskChassis */
}

/* USER CODE BEGIN Header_SekiroTask */
/**
* @brief Function implementing the Sekirotask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SekiroTask */
__weak void SekiroTask(void *argument)
{
  /* USER CODE BEGIN SekiroTask */
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END SekiroTask */
}

/* USER CODE BEGIN Header_Testtask */
/**
* @brief Function implementing the Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Testtask */
__weak void Testtask(void *argument)
{
  /* USER CODE BEGIN Testtask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Testtask */
}

/* USER CODE BEGIN Header_TaskTrajctory */
/**
* @brief Function implementing the TaskTraj thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskTrajctory */
__weak void TaskTrajctory(void *argument)
{
  /* USER CODE BEGIN TaskTrajctory */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskTrajctory */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

