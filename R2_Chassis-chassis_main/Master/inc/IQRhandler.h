#ifndef __IQRHANDLER_H__
#define __IQRHANDLER_H__

#include "includes.h"
#include "tim.h"
#include "FD_Canqueue.h"
#include "DebugCtrl.h"

extern __RAM_D3_ ALIGN_32B uint8_t rx_temp2;
extern __RAM_D2_ ALIGN_32B uint8_t rx_temp6;
extern __RAM_D2_ ALIGN_32B uint8_t rx_temp3;
extern __RAM_D2_ ALIGN_32B uint8_t rx_temp4;
extern __RAM_D2_ ALIGN_32B uint8_t USART6_RxBuffer[32];
extern __RAM_D2_ ALIGN_32B uint8_t UART4_RxBuffer[32];
extern __RAM_D2_ ALIGN_32B uint8_t USART2_RxBuffer[32];
extern __RAM_D2_ ALIGN_32B uint8_t USART3_RxBuffer[32];

extern GPIO_PinState Hallelement;
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
#endif /* __IQRHANDLER_H__ */
