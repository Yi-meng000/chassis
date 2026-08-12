#pragma once

#include "includes.h"
#define BEEP_PORT GPIOB

#define BEEP_PIN GPIO_PIN_0

#define BEEP_ON HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET)
#define BEEP_OFF HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET)

void BEEP_Alarm(uint8_t time);
void BEEP_Start(void);
void LED_Flow(void);
