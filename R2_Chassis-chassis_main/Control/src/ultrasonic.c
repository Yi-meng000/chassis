#include "ultrasonic.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

float ultrasonic_height_F = 0;			// 底盘前面的超声波，检测是否能下前腿
float u_height_last_F = 0;
float u_height_change_F = 0;
float ultrasonic_height_UP_B = 0;		// 中间左边靠后的超声波，检测是否能收后腿
float u_height_last_UP_B = 0;
float u_height_change_UP_B = 0;

static TIM_HandleTypeDef htim5;

Ultrasonic_t ultrasonic_E =
{
    .trig_port = GPIOD,
    .trig_pin  = GPIO_PIN_5,
    .echo_port = GPIOD,
    .echo_pin  = GPIO_PIN_6,
};

Ultrasonic_t ultrasonic_F =
{
    .trig_port = GPIOD,
    .trig_pin  = GPIO_PIN_15,
    .echo_port = GPIOD,
    .echo_pin  = GPIO_PIN_14,
};

/* 使用 TIM5(1MHz) 做简易微秒延时：等待计数差达到指定微秒数 */
static void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);
    while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim5) - start) < us) {}
}


static void TIM5_Init_1MHz(void)
{
 uint32_t tim_clk;

    __HAL_RCC_TIM5_CLK_ENABLE();

    /* TIM5 clock = APB1 timer clock */
    if (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1)
    {
        tim_clk = HAL_RCC_GetPCLK1Freq() * 2;
    }
    else
    {
        tim_clk = HAL_RCC_GetPCLK1Freq();
    }

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = (tim_clk / 1000000U) - 1;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim5);
    HAL_TIM_Base_Start(&htim5);
}


void Ultrasonic_Init(void)
{
    TIM5_Init_1MHz();

    /* 复位所有 TRIG 引脚 */
    HAL_GPIO_WritePin(ultrasonic_E.trig_port, ultrasonic_E.trig_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ultrasonic_F.trig_port, ultrasonic_F.trig_pin, GPIO_PIN_RESET);

}

void Ultrasonic_Trigger(Ultrasonic_t *u)
{
    u->measure_done = 0;
    u->echo_active  = 0;

    HAL_GPIO_WritePin(u->trig_port, u->trig_pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(u->trig_port, u->trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(u->trig_port, u->trig_pin, GPIO_PIN_RESET);
}

float Ultrasonic_GetDistance(Ultrasonic_t *u)
{
    return u->last_distance_cm;
}

BaseType_t Ultrasonic_WaitForMeasure(Ultrasonic_t *u, TickType_t ticksToWait)
{
    TickType_t start = xTaskGetTickCount();

    while (!u->measure_done)
    {
        if ((xTaskGetTickCount() - start) >= ticksToWait)
        {
            return pdFALSE;
        }
        vTaskDelay(1);
    }
    u->measure_done = 0;
    return pdTRUE;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Ultrasonic_t *u = NULL;
		
		if (GPIO_Pin == ultrasonic_E.echo_pin)
        u = &ultrasonic_E;
		else if (GPIO_Pin == ultrasonic_F.echo_pin)
        u = &ultrasonic_F;
    else
        return;

    if (HAL_GPIO_ReadPin(u->echo_port, u->echo_pin) == GPIO_PIN_SET)
    {
        u->echo_start  = __HAL_TIM_GET_COUNTER(&htim5);
        u->echo_active = 1;
    }
    else
    {
        if (!u->echo_active) return;

        u->echo_end = __HAL_TIM_GET_COUNTER(&htim5);

        uint32_t pulse_us =
            (u->echo_end >= u->echo_start) ?
            (u->echo_end - u->echo_start) :
            (0xFFFFFFFFU - u->echo_start + u->echo_end + 1U);

        u->last_distance_cm = pulse_us / 58.0f;
        u->measure_done = 1;
        u->echo_active  = 0;
    }
}

