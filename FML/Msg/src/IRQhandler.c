#include "IRQhandler.h"

__RAM_D1_ ALIGN_32B uint8_t rx_temp1[RXBufferNum] = {0};
__RAM_D3_ ALIGN_32B uint8_t rx_temp2 = 0;
__RAM_D1_ ALIGN_32B uint8_t rx_temp3 = 0;
__RAM_D3_ ALIGN_32B uint8_t rx_temp6 = 0;
__RAM_D2_ ALIGN_32B uint8_t rx_temp4 = 0;
__RAM_D2_ ALIGN_32B uint8_t rx_temp9[ROS_PACK_LEN] = {0};
uint8_t rx_ros[16] = {0};

/*
 * UART9 is a byte stream, not a packet transport.  A lost byte would make a
 * fixed 16-byte receive permanently misaligned, so retain a possible
 * header after a bad frame and only receive the bytes that are still missing.
 */
static void UART9_StartReceive(UART_HandleTypeDef *huart, uint16_t bytes_kept)
{
    if ((bytes_kept >= ROS_PACK_LEN) ||
        (HAL_UART_Receive_IT(huart, &rx_temp9[bytes_kept], ROS_PACK_LEN - bytes_kept) != HAL_OK))
    {
        Error_Handler();
    }
}

static uint16_t UART9_Resync(void)
{
    int16_t header_index;

    /*
     * Keep the earliest candidate rather than skipping a valid header that
     * appears later in this window.  For example, a frame ending in
     * 0x5A, 0xA5 keeps that last A5 and receives the remaining 15 bytes of the
     * following frame.
     */
    for (header_index = 1; header_index < ROS_PACK_LEN; ++header_index)
    {
        if (rx_temp9[header_index] == 0xA5U)
        {
            uint16_t bytes_kept = ROS_PACK_LEN - (uint16_t)header_index;
            memmove(rx_temp9, &rx_temp9[header_index], bytes_kept);
            return bytes_kept;
        }
    }

    return 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&rx_temp2), 1);
        if (HAL_UART_Receive_DMA(huart, &rx_temp2, 1) != HAL_OK)
            Error_Handler();
//        Debug_Receive(&DebugMsg, rx_temp2);
        // ✓ 使DCache失效,从RAM读取DMA写入的新数据
    }
    if (huart->Instance == USART1)
    {
        //       SCB_InvalidateDCache_by_Addr((uint32_t *)(rx_temp1),RXBufferNum);
        // //      if(HAL_UART_Receive_DMA(huart,rx_temp1,RXBufferNum) != HAL_OK)
        // //        Error_Handler();
        // 				SensorUsartReceiveHandler(rx_temp1,RXBufferNum); //雷达接收
    }
    if (huart->Instance == USART3)
    {
		SCB_InvalidateDCache_by_Addr((uint32_t *)(&rx_temp3), 1);
        if (HAL_UART_Receive_DMA(huart, &rx_temp3, 1) != HAL_OK)
            Error_Handler();
        RobotCom_ReceiveHandler(&RobotComMsg, rx_temp3);
        //			Route_ReceiveByte(&RouteMsg,rx_temp3);
    }
    if (huart->Instance == UART4)
    {
        if(HAL_UART_Receive_IT(huart,&rx_temp4,1) != HAL_OK)
          Error_Handler();
        RobotCom_TraceOnReceive(&MatchTrace,rx_temp4);
    }
    if (huart->Instance == USART6)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&rx_temp6), 1);
        if (HAL_UART_Receive_DMA(huart, &rx_temp6, 1) != HAL_OK)
            Error_Handler();
        Camera_LightReceiveHandler(&CameraLightPack,rx_temp6);
    }
    if (huart->Instance == UART9)
    {
        if ((rx_temp9[0] == 0xA5U) && (rx_temp9[ROS_PACK_LEN - 1U] == 0x5AU))
        {
//          RobotCom_QrcodeScan(&RobotRxmsg, rx_temp9);
            ROS2STM_Comtest(&RosComPack, rx_temp9);
            UART9_StartReceive(huart, 0U);
        }
        else
        {
            UART9_StartReceive(huart, UART9_Resync());
        }
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
    // if(huart->Instance == USART1)
    //     HAL_UART_Receive_DMA(huart,rx_temp1,RXBufferNum);
    if (huart->Instance == USART2)
        HAL_UART_Receive_DMA(huart, &rx_temp2, 1);
		else if(huart->Instance == USART3)
				HAL_UART_Receive_DMA(huart, &rx_temp3, 1);
        else if(huart->Instance == UART9)
                UART9_StartReceive(huart, 0U);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        USER_USART_GO_RxHandler(huart, Size, 0);
    }
    if (huart->Instance == USART1)
    {
        SENSOR_USART_RxHandler(&huart1, Size);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        U3_RS485_RxMode();
    }
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
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    FDCAN_RxHeaderTypeDef Rxheader;
    uint8_t Rxdata[8] = {0};

    /* USER CODE END Callback 0 */

    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM2)
    {
        CAN_DequeueTx(&CAN1_Txqueue);
        CAN_DequeueTx(&CAN2_Txqueue);

        if (CAN_DequeueRx(&CAN2_Rxqueue, &Rxheader, Rxdata))
        {
            if (Rxheader.Identifier < 0x05020100)
                chassis_ReceiveHandler(&Chassis, Rxheader, Rxdata);
            else
                Actuator_ReceiveHandler(&Actparam, Rxdata, Rxheader);
        }
        // if(CAN_DequeueRx(&CAN1_Rxqueue,&Rxheader,Rxdata))
        // {
        //   SENSOR_RecvHandler(Rxheader,Rxdata);
        // }
    }
    if (htim->Instance == TIM3)
    {
#if USE_ZMDR
        static uint8_t Zdrive_count = 0;
        if (Zdrive_count++ > 10)
        {
            ZdriveFunc();
            Zdrive_count = 0;
        }
#endif
#if USE_VESC
        static uint8_t Vesc_count = 0;
        if (Vesc_count++ > 10)
        {
            VescFunc();
            Vesc_count = 0;
        }
#endif
#if USE_DJ
        DJFunc();
#endif
#if USE_UNITREE
        Unitree_GO_Func();
#endif
    }
    /* USER CODE END Callback 1 */
}
