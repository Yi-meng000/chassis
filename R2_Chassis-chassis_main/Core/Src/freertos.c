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
#include "LED.h"
#include "ZDrive.h"
#include "VescMotor.h"
#include "DJmotor.h"
#include "chassisComm.h"
#include "DebugCtrl.h"
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
/* Definitions for Led_Task */
osThreadId_t Led_TaskHandle;
const osThreadAttr_t Led_Task_attributes = {
  .name = "Led_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Com_Task */
osThreadId_t UART_Com_TaskHandle;
const osThreadAttr_t UART_Com_Task_attributes = {
  .name = "UART_Com_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskChassis */
osThreadId_t taskChassisHandle;
const osThreadAttr_t taskChassis_attributes = {
  .name = "taskChassis",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Climbtask */
osThreadId_t ClimbtaskHandle;
const osThreadAttr_t Climbtask_attributes = {
  .name = "Climbtask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Ultrasonic_task */
osThreadId_t Ultrasonic_taskHandle;
const osThreadAttr_t Ultrasonic_task_attributes = {
  .name = "Ultrasonic_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Guiji */
osThreadId_t GuijiHandle;
const osThreadAttr_t Guiji_attributes = {
  .name = "Guiji",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tinyf */
osThreadId_t tinyfHandle;
const osThreadAttr_t tinyf_attributes = {
  .name = "tinyf",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Led_Task_Func(void *argument);
void UART_Com_Fuc(void *argument);
void TaskChassis(void *argument);
void Climbsteps(void *argument);
void UltrasonicTask(void *argument);
void guiji(void *argument);
void TINYF(void *argument);

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
  /* creation of Led_Task */
  Led_TaskHandle = osThreadNew(Led_Task_Func, NULL, &Led_Task_attributes);

  /* creation of UART_Com_Task */
  UART_Com_TaskHandle = osThreadNew(UART_Com_Fuc, NULL, &UART_Com_Task_attributes);

  /* creation of taskChassis */
  taskChassisHandle = osThreadNew(TaskChassis, NULL, &taskChassis_attributes);

  /* creation of Climbtask */
  ClimbtaskHandle = osThreadNew(Climbsteps, NULL, &Climbtask_attributes);

  /* creation of Ultrasonic_task */
  Ultrasonic_taskHandle = osThreadNew(UltrasonicTask, NULL, &Ultrasonic_task_attributes);

  /* creation of Guiji */
  GuijiHandle = osThreadNew(guiji, NULL, &Guiji_attributes);

  /* creation of tinyf */
  tinyfHandle = osThreadNew(TINYF, NULL, &tinyf_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Led_Task_Func */
/**
  * @brief  Function implementing the Led_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Led_Task_Func */
__weak void Led_Task_Func(void *argument)
{
  /* USER CODE BEGIN Led_Task_Func */
  /* Infinite loop */
  for(;;)
  {
		LED_Flow();

  }
  /* USER CODE END Led_Task_Func */
}

/* USER CODE BEGIN Header_UART_Com_Fuc */
/**
* @brief Function implementing the UART_Com_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Com_Fuc */
__weak void UART_Com_Fuc(void *argument)
{
  /* USER CODE BEGIN UART_Com_Fuc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART_Com_Fuc */
}

/* USER CODE BEGIN Header_TaskChassis */
/**
* @brief Function implementing the taskChassis thread.
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

/* USER CODE BEGIN Header_Climbsteps */
/**
* @brief Function implementing the Climbtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Climbsteps */
__weak void Climbsteps(void *argument)
{
  /* USER CODE BEGIN Climbsteps */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Climbsteps */
}

/* USER CODE BEGIN Header_UltrasonicTask */
/**
* @brief Function implementing the Ultrasonic_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasonicTask */
__weak void UltrasonicTask(void *argument)
{
  /* USER CODE BEGIN UltrasonicTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UltrasonicTask */
}

/* USER CODE BEGIN Header_guiji */
/**
* @brief Function implementing the Guiji thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_guiji */
__weak void guiji(void *argument)
{
  /* USER CODE BEGIN guiji */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END guiji */
}

/* USER CODE BEGIN Header_TINYF */
/**
* @brief Function implementing the tinyf thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TINYF */
__weak void TINYF(void *argument)
{
  /* USER CODE BEGIN TINYF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TINYF */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

