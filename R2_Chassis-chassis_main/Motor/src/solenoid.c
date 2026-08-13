#include "solenoid.h"
#include "FD_Canqueue.h"

#define SOLENOID_CAN_STD_ID 0x123U

/* solenoid 数据：8字节标准帧，ID=0x123，data[0]=控制字节 */
/* data[0] 低4位: bit0=CH1(辅助轮), bit2=CH3(前气缸), bit3=CH4(后气缸) */
/* 通过 CAN1_Txqueue 队列发送（FDCAN1 PD0/PD1），在 TIM2 中断中出队发送 */

static uint8_t g_last_solenoid_cmd = 0xFF;

void solenoid_init(uint8_t usart_channel)
{
    (void)usart_channel;
    g_last_solenoid_cmd = 0xFF;   /* 确保首次不同 */
    solenoid_on(usart_channel, 0);/* 初始发送0，确保电磁阀关断 */
    solenoid_flag = 0;
}

void solenoid_on(uint8_t usart_channel, uint8_t cmd)
{
    (void)usart_channel;
    uint8_t data = (uint8_t)(cmd & 0x0F);

    if (data == g_last_solenoid_cmd)
    {
        return;                   /* 数据未变化，跳过发送 */
    }
    g_last_solenoid_cmd = data;

    if (CAN_Queue_IfFull(&CAN1_Txqueue))
    {
        return;
    }

    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = SOLENOID_CAN_STD_ID;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 8;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].IDE = FDCAN_STANDARD_ID;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = data;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1] = 0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[2] = 0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[3] = 0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[4] = 0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[5] = 0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[6] = 0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[7] = 0;
    CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
}
