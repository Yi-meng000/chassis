#include "RadarMsg.h"
#include <string.h>
SENSORUSART_MSG SensorUsart_Msg = {0};
SENSOR_RXPACK Sensor_RxPack = {0};
SENSOR_TXPACK Sensor_TxPack = {0};
__RAM_D2_ ALIGN_32B uint8_t Radar_RxBuff[2][LASER_PACK_SIZE] = {0};
u8 LaserRxData[LASER_PACK_SIZE]; // 提取缓冲区的数据存入此数组
// 无校验
// 空闲中断回调函数中调用
void SENSOR_USART_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, LASER_RXBUFF);
        if (Size == LASER_PACK_SIZE)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)(Radar_RxBuff[0]),
                                         LASER_PACK_SIZE);
            memcpy(LaserRxData, Radar_RxBuff[0], LASER_PACK_SIZE);
        }
    }
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, LASER_RXBUFF);
        if (Size == LASER_PACK_SIZE)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)(Radar_RxBuff[1]),
                                         LASER_PACK_SIZE);
            memcpy(LaserRxData, Radar_RxBuff[1], LASER_PACK_SIZE);
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void LaserUsartDeal(SENSOR_RXPACK *Sensor_RxPack)
{
    if (LaserRxData[0] == DEBUG_PREFIX_1 && LaserRxData[1] == DEBUG_PREFIX_2)
    {
        if (LaserRxData[LASER_PACK_SIZE - 2] == DEBUG_SUFFIX_1 && LaserRxData[LASER_PACK_SIZE - 1] == DEBUG_SUFFIX_2)
        {
            u8 type = LaserRxData[2];
            int16_t rxindex = 2;
            if (type == 0x01)
            {
#if Laser_RX_BYTE_NUM
                for (u8 i = 0; i < Laser_RX_BYTE_NUM; i++)
                {
                    Sensor_RxPack->Laser_RxPack.bytes[i] = LaserRxData[rxindex++];
                }
#endif
#if Laser_RX_FLOAT_NUM
                for (int i = 0; i < Laser_RX_FLOAT_NUM; i++)
                {
                    memcpy(&(Sensor_RxPack->Laser_RxPack.floats[i]), &(LaserRxData[rxindex]),
                           sizeof(float));
                    rxindex += 4;
                }
#endif
                if (rxindex != (Laser_RX_BYTE_NUM + Laser_RX_FLOAT_NUM * 4))
                    return;
            }
        }
    }
    return;
}

void LaserRelocation(SENSORUSART_MSG *msg, SENSOR_TXPACK *txPack, uint8_t reloc_mode, float tmp_x, float tmp_y, float tmp_angle)
{
    txPack->bytes[0] = reloc_mode;

    txPack->floats[0] = tmp_x / 1000.f;
    txPack->floats[1] = tmp_y / 1000.f;
    txPack->floats[2] = tmp_angle;
	txPack->floats[3] = 0;

    u16 txIndex = 0;

    msg->TxData[txIndex++] = DEBUG_PREFIX_1;
    msg->TxData[txIndex++] = DEBUG_PREFIX_2;
	
		msg->TxData[txIndex++] = 0x01;
		msg->TxData[txIndex++] = 0x78;
		msg->TxData[txIndex++] = 0x13;
//#if Laser_TX_BYTE_NUM
//    for (uint8_t i = 0; i < Laser_TX_BYTE_NUM; i++)
//    {
//        msg->TxData[txIndex++] = txPack->bytes[i];
//    }
//#endif
//#if Laser_TX_FLOAT_NUM
//    for (uint8_t i = 0; i < Laser_TX_FLOAT_NUM; i++)
//    {
//        memcpy(msg->TxData + txIndex, txPack->floats + i, sizeof(float));
//        txIndex += 4;
//    }
//#endif
    msg->TxData[txIndex++] = DEBUG_SUFFIX_1;
    msg->TxData[txIndex++] = DEBUG_SUFFIX_2;
    SCB_CleanDCache_by_Addr((uint32_t *)(msg->TxData), txIndex);
    HAL_UART_Transmit_IT(&huart1, msg->TxData, txIndex);
}
