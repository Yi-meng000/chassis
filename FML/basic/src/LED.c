#include "LED.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
void LED_Flow(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    osDelay(200);
}

void BEEP_Start(void)
{
    BEEP_ON;
    HAL_Delay(50);
    BEEP_OFF;
    HAL_Delay(50);
    BEEP_ON;
    HAL_Delay(50);
    BEEP_OFF;
    HAL_Delay(50);
}

/**
 * @brief 蜂鸣器报警函数，按固定模式报警，重复 time 次。
 * @param time 蜂鸣器报警次数
 */
void BEEP_Alarm(uint8_t time)
{
    do
    {
        BEEP_ON;
        osDelay(40);
        BEEP_OFF;
        osDelay(60);
        time--;
    } while (0 != time);
}
