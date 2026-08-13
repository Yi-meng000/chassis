#pragma once
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;

    uint32_t echo_start;
    uint32_t echo_end;
    uint8_t  echo_active;
    uint8_t  measure_done;

    float last_distance_cm;
} Ultrasonic_t;

/* 两个实例 */
extern Ultrasonic_t ultrasonic_E;
extern Ultrasonic_t ultrasonic_F;


extern float ultrasonic_height_F;
extern float u_height_last_F;
extern float u_height_change_F;
extern float ultrasonic_height_UP_B;
extern float u_height_last_UP_B;
extern float u_height_change_UP_B;

/* 接口 */
extern BaseType_t Ultrasonic_WaitForMeasure(Ultrasonic_t *u, TickType_t ticksToWait);
extern void Ultrasonic_Init(void);
extern void Ultrasonic_Trigger(Ultrasonic_t *u);
extern BaseType_t Ultrasonic_WaitDone(Ultrasonic_t *u, TickType_t timeout);
extern float Ultrasonic_GetDistance(Ultrasonic_t *u);
