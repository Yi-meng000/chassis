#include "IQRhandler.h"
#include "Ultrasound_new.h"
#include "TinyF.h"
__RAM_D3_ ALIGN_32B uint8_t rx_temp2 = 0;
__RAM_D2_ ALIGN_32B uint8_t rx_temp6 = 0;
__RAM_D2_ ALIGN_32B uint8_t rx_temp3 = 0;

__RAM_D2_ ALIGN_32B uint8_t USART6_RxBuffer[32] = {0};
__RAM_D2_ ALIGN_32B uint8_t UART4_RxBuffer[32] = {0};
__RAM_D2_ ALIGN_32B uint8_t USART2_RxBuffer[32] = {0};
__RAM_D2_ ALIGN_32B uint8_t USART3_RxBuffer[32] = {0};

GPIO_PinState Hallelement = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    //				SCB_InvalidateDCache_by_Addr((uint32_t *)(&rx_temp2),1);
    ////        HAL_UART_Receive_DMA(&huart2,&rx_temp2,1);
    //        Debug_Receive(&DebugMsg,rx_temp2);
  }
  if (huart->Instance == USART6)
  {
    // HAL_UART_Receive_IT(&huart6,&rx_temp6,1);
    // MINIF_Processing_Data(rx_temp6);
  }
  else if (huart->Instance == USART3)
  {
    //			UltraSound_Receive(rx_temp3,&UltraSound_front);
    //			HAL_UART_Receive_IT(&huart3,&rx_temp3,1);
  }
}

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  FDCAN_RxHeaderTypeDef Rxheader;
  uint8_t Rxdata[8] = {0};
  if (htim->Instance == TIM2)
  {
    CAN_DequeueTx(&CAN1_Txqueue);
    CAN_DequeueTx(&CAN2_Txqueue);
    CAN_DequeueTx(&CAN3_Txqueue);
    if (CAN_DequeueRx(&CAN1_Rxqueue, &Rxheader, Rxdata))
    {
      ChassisFunc(Rxheader, Rxdata);
    }
  }
  else if (htim->Instance == TIM3)
  {
    CAN_DequeueTx(&CAN2_Txqueue);
    if (CAN_DequeueRx(&CAN2_Rxqueue, &Rxheader, Rxdata))
    {
      ZdriveReceive(Rxheader, Rxdata);
    }
#if USE_ZMDR
    static uint8_t Zdrive_cnt = 0;
    if (++Zdrive_cnt >= 10)
    {
      Zdrive_cnt = 0;
      ZdriveFunc();
    }
#endif
#if USE_VESC
    static uint8_t VescCount = 0;
    if (++VescCount >= 10)
    {
      VescCount = 0;
      VescFunc();
    }
#endif
    if (CAN_DequeueRx(&CAN3_Rxqueue, &Rxheader, Rxdata))
    {
      ZdriveReceive(Rxheader, Rxdata);
    }
  }
  else if (htim->Instance == TIM4)
  {
    CAN_DequeueTx(&CAN3_Txqueue);
    if (CAN_DequeueRx(&CAN2_Rxqueue, &Rxheader, Rxdata))
    {
      ZdriveReceive(Rxheader, Rxdata);
    }
#if USE_DJ
    DJFunc();
#endif
  }
  else if (htim->Instance == TIM5)
  {
    up_down_track();
  }
  /* USER CODE END Callback 1 */
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef Rxheader;
  uint8_t Rx_data[8] = {0};

  if (hfdcan == &hfdcan1)
  {
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &Rxheader, Rx_data);
    CAN_Enqueue(&CAN1_Rxqueue, Rxheader, Rx_data);
  }
  else if (hfdcan == &hfdcan2)
  {
    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &Rxheader, Rx_data);
    CAN_Enqueue(&CAN2_Rxqueue, Rxheader, Rx_data);
  }
  else if (hfdcan == &hfdcan3)
  {
    HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &Rxheader, Rx_data);
    CAN_Enqueue(&CAN3_Rxqueue, Rxheader, Rx_data);
  }
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  FDCAN_RxHeaderTypeDef Rxheader;
  uint8_t Rx_data[8] = {0};

  if (hfdcan == &hfdcan1)
  {
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &Rxheader, Rx_data);
    CAN_Enqueue(&CAN1_Rxqueue, Rxheader, Rx_data);
  }
  else if (hfdcan == &hfdcan2)
  {
    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &Rxheader, Rx_data);
    CAN_Enqueue(&CAN2_Rxqueue, Rxheader, Rx_data);
  }
  else if (hfdcan == &hfdcan3)
  {
    HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &Rxheader, Rx_data);
    CAN_Enqueue(&CAN3_Rxqueue, Rxheader, Rx_data);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->ErrorCode & HAL_UART_ERROR_ORE)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  if (huart->ErrorCode & HAL_UART_ERROR_FE)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
  }
  if (huart->Instance == USART2)
    //      HAL_UART_Receive_DMA(huart,&rx_temp2,1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2_RxBuffer, 32);
  else if (huart->Instance == USART6)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_RxBuffer, 32);
  else if (huart->Instance == USART3)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_RxBuffer, 32);
  else if (huart->Instance == UART4)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_RxBuffer, 32);
}

// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//     if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//     {
//         Hallelement= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
//     }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t recv_len;
  uint8_t port_id;

  if (huart->Instance == UART4)
  {
    port_id = TINYF_SOL;
    SCB_InvalidateDCache_by_Addr((uint32_t *)UART4_RxBuffer, Size);
    for (uint16_t i = 0; i < Size; i++)
    {
      MINIF_Processing_Data(port_id, UART4_RxBuffer[i]);
    }
    HAL_UART_DMAStop(&huart4);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_RxBuffer, 32);
  }
  else if (huart->Instance == USART2)
  {
    port_id = TINYF_BACK;
    SCB_InvalidateDCache_by_Addr((uint32_t *)USART2_RxBuffer, Size);
    for (uint16_t i = 0; i < Size; i++)
    {
      MINIF_Processing_Data(port_id, USART2_RxBuffer[i]);
    }
    HAL_UART_DMAStop(&huart2);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2_RxBuffer, 32);
  }
  else if (huart->Instance == USART3)
  {
    port_id = TINYF_FRONT;
    SCB_InvalidateDCache_by_Addr((uint32_t *)USART3_RxBuffer, Size);
    for (uint16_t i = 0; i < Size; i++)
    {
      MINIF_Processing_Data(port_id, USART3_RxBuffer[i]);
    }
    HAL_UART_DMAStop(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_RxBuffer, 32);
  }
  else if (huart->Instance == USART6)
  {
    port_id = TINYF_MID;
    SCB_InvalidateDCache_by_Addr((uint32_t *)USART6_RxBuffer, Size);
    for (uint16_t i = 0; i < Size; i++)
    {
      MINIF_Processing_Data(port_id, USART6_RxBuffer[i]);
    }
    HAL_UART_DMAStop(&huart6);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_RxBuffer, 32);
  }
}
