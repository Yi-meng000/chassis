#include "my_Unitree.h"

__RAM_D1_ ALIGN_32B u8 GO_rx_buf[RS_num][2][RS485_RXLENGTH] = {0};
__RAM_D2_ ALIGN_32B MotorCmd_t control_cmd = {0};
MotorData_t control_data[RS_num] = {0};
Go_motor my_GO_motor[RS_num][GO_num] = {0};
uint8_t GoCount = 0;
HAL_StatusTypeDef tx_res[RS_num];
bool Unitree_set_zero_all_flag = false;
bool Unitree_All_Enable_flag = false;

void my_Unitree_Init()
{
    U3_RS485_RxMode();
    MotorCmd_t cmd = {.K_P = 4.f, .K_W = 0.25f};
    for (int i = 0; i < RS_num; i++)
    {
        for (int j = 0; j < GO_num; j++)
        {
            my_GO_motor[i][j].cmd = cmd;
            my_GO_motor[i][j].cmd.id = j;
            my_GO_motor[i][j].data.motor_id = j;
            my_GO_motor[i][j].set_zero_Pos = 0;
            my_Unitree_set_zero(i, j);
        }
    }
}
void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_IDLE;
    huart->RxXferSize = DataLength;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)DstAddress, (uint32_t)SecondMemAddress, DataLength);
}
void my_Unitree_set_zero(int RS_id, int GO_id)
{
    if (RS_id < RS_num)
    {
        if (GO_id < GO_num)
        {
            my_GO_motor[RS_id][GO_id].set_zero_Pos += my_GO_motor[RS_id][GO_id].data.Pos;
            my_GO_motor[RS_id][GO_id].cmd.Pos = 0;
            my_GO_motor[RS_id][GO_id].data.Pos = 0;
        }
        else
        {
            for (int i = 0; i < GO_num; i++)
            {
                my_GO_motor[RS_id][i].set_zero_Pos += my_GO_motor[RS_id][i].data.Pos;
                my_GO_motor[RS_id][i].cmd.Pos = 0;
                my_GO_motor[RS_id][i].data.Pos = 0;
            }
        }
    }
    else
    {
        for (int i = 0; i < RS_num; i++)
        {
            if (GO_id < GO_num)
            {
                my_GO_motor[i][GO_id].set_zero_Pos += my_GO_motor[i][GO_id].data.Pos;
                my_GO_motor[i][GO_id].cmd.Pos = 0;
                my_GO_motor[i][GO_id].data.Pos = 0;
            }
            else
            {
                for (int j = 0; j < GO_num; j++)
                {
                    my_GO_motor[i][j].set_zero_Pos += my_GO_motor[i][j].data.Pos;
                    my_GO_motor[i][j].cmd.Pos = 0;
                    my_GO_motor[i][j].data.Pos = 0;
                }
            }
        }
    }
}

void Unitree_enable(int RS_id, int GO_id)
{
    if (RS_id < RS_num)
    {
        if (GO_id < GO_num)
        {
            my_GO_motor[RS_id][GO_id].cmd.mode = 1;
            if (fabsf(my_GO_motor[RS_id][GO_id].cmd.K_W) < 0.01f)
                my_GO_motor[RS_id][GO_id].cmd.K_W = 0.2f;
        }
        else
        {
            for (int i = 0; i < GO_num; i++)
            {
                my_GO_motor[RS_id][i].cmd.mode = 1;
                if (fabsf(my_GO_motor[RS_id][i].cmd.K_W) < 0.01f)
                    my_GO_motor[RS_id][i].cmd.K_W = 0.2f;
            }
        }
    }
    else
    {
        for (int i = 0; i < RS_num; i++)
        {
            if (GO_id < GO_num)
            {
                my_GO_motor[i][GO_id].cmd.mode = 1;
                if (fabsf(my_GO_motor[i][GO_id].cmd.K_W) < 0.01f)
                    my_GO_motor[i][GO_id].cmd.K_W = 0.2f;
            }
            else
            {
                for (int j = 0; j < GO_num; j++)
                {
                    my_GO_motor[i][j].cmd.mode = 1;
                    if (fabsf(my_GO_motor[i][j].cmd.K_W) < 0.01f)
                        my_GO_motor[i][j].cmd.K_W = 0.2f;
                }
            }
        }
    }
}

void Unitree_disable(int RS_id, int GO_id)
{
    if (RS_id < RS_num)
    {
        if (GO_id < GO_num)
        {
            my_GO_motor[RS_id][GO_id].cmd.mode = 0;
        }
        else
        {
            for (int i = 0; i < GO_num; i++)
            {
                my_GO_motor[RS_id][i].cmd.mode = 0;
            }
        }
    }
    else
    {
        for (int i = 0; i < RS_num; i++)
        {
            if (GO_id < GO_num)
            {
                my_GO_motor[i][GO_id].cmd.mode = 0;
            }
            else
            {
                for (int j = 0; j < GO_num; j++)
                {
                    my_GO_motor[i][j].cmd.mode = 0;
                }
            }
        }
    }
}

static void U3_Unitree_Function(int index)
{
    control_cmd = my_GO_motor[U3_RS_Channel][index].cmd;
    control_cmd.Pos +=
        my_GO_motor[U3_RS_Channel][index].set_zero_Pos; // 实际设置是控制量加上初始偏移量
    modify_data(&control_cmd);
    U3_RS485_TxMode();
    SCB_CleanDCache_by_Addr((uint32_t *)(&control_cmd), sizeof(MotorCmd_t));
    tx_res[0] = HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&(control_cmd.motor_send_data),
                                      sizeof(control_cmd.motor_send_data));
    // 接收数据处理
    extract_data(control_data + U3_RS_Channel);
    my_GO_motor[U3_RS_Channel][control_data[U3_RS_Channel].motor_id].data =
        control_data[U3_RS_Channel];
    my_GO_motor[U3_RS_Channel][control_data[U3_RS_Channel].motor_id].data.Pos -=
        my_GO_motor[0][control_data[0].motor_id].set_zero_Pos;
}
void USER_USART_GO_RxHandler(UART_HandleTypeDef *huart, uint16_t Size, uint8_t rs485_index)
{
    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        __HAL_DMA_SET_COUNTER(huart->hdmarx, RS485_RXBUF);

        if (Size == RS485_RXLENGTH)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)(GO_rx_buf[rs485_index][0]), RS485_RXLENGTH);
            memcpy(&(control_data[rs485_index].motor_recv_data), GO_rx_buf[rs485_index][0],
                   RS485_RXLENGTH);
        }
    }
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        __HAL_DMA_SET_COUNTER(huart->hdmarx, RS485_RXBUF);

        if (Size == RS485_RXLENGTH)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)(GO_rx_buf[rs485_index][1]), RS485_RXLENGTH);
            memcpy(&(control_data[rs485_index].motor_recv_data), GO_rx_buf[rs485_index][1],
                   RS485_RXLENGTH);
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);
}
void Unitree_GO_Func()
{
    if (Unitree_set_zero_all_flag)
    {
        my_Unitree_set_zero(RS_num, GO_num);
        Unitree_set_zero_all_flag = false;
    }
    if (Unitree_All_Enable_flag)
    {
        Unitree_enable(RS_num, GO_num);
        Unitree_All_Enable_flag = false;
    }
    for (int i = 0; i < RS_num; ++i)
    {
        for (int j = 0; j < GO_num; ++j)
        {
            if (my_GO_motor[i][j].Enable)
            {
                Unitree_enable(i, j);
            }
            if (my_GO_motor[i][j].SetZero)
            {
                my_Unitree_set_zero(i, j);
            }
        }
    }
    U3_Unitree_Function(GoCount);
    GoCount++;
    if (GoCount > (GO_num - 1))
    {
        GoCount = 0;
    }
}
