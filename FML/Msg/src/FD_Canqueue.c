#include "FD_Canqueue.h"

FDCAN_SendQueueType CAN1_Txqueue, CAN1_Rxqueue, CAN2_Txqueue, CAN2_Rxqueue;
bool Can1FullFlag = 0;
bool Can2FullFlag = 0;
void CAN_InitSendQueue(void)
{
    CAN1_Rxqueue.Front = CAN1_Rxqueue.Rear = 0;
    CAN1_Txqueue.Front = CAN1_Txqueue.Rear = 0;
    CAN2_Rxqueue.Front = CAN2_Rxqueue.Rear = 0;
    CAN2_Txqueue.Front = CAN2_Txqueue.Rear = 0;
    CAN1_Rxqueue.Canx = FDCAN1;
    CAN1_Txqueue.Canx = FDCAN1;
    CAN2_Rxqueue.Canx = FDCAN2;
    CAN2_Txqueue.Canx = FDCAN2;
    for (int i = 0; i < FDCAN_QUEUESIZE; i++)
    {
        CAN1_Rxqueue.FDCAN_DataSend[i].IDE = FDCAN_STANDARD_ID;
        CAN1_Txqueue.FDCAN_DataSend[i].IDE = FDCAN_STANDARD_ID;
        CAN2_Rxqueue.FDCAN_DataSend[i].IDE = FDCAN_STANDARD_ID;
        CAN2_Txqueue.FDCAN_DataSend[i].IDE = FDCAN_STANDARD_ID;
    }
    return;
}
bool CAN_Queue_IfEmpty(FDCAN_SendQueueType *queue)
{
    return (queue->Front == queue->Rear);
}
bool CAN_Queue_IfFull(FDCAN_SendQueueType *queue)
{
    return ((queue->Rear + 1) % FDCAN_QUEUESIZE == queue->Front);
}
void CAN_DequeueTx(FDCAN_SendQueueType *queue)
{
    if (CAN_Queue_IfEmpty(queue))
        return;
    FDCAN_TxHeaderTypeDef txmessage;
    uint8_t Tx_msg[8] = {0};

    txmessage.TxFrameType = FDCAN_DATA_FRAME;
    txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txmessage.BitRateSwitch = FDCAN_BRS_OFF;
    txmessage.FDFormat = FDCAN_CLASSIC_CAN;
    txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txmessage.MessageMarker = 0;
    txmessage.IdType = queue->FDCAN_DataSend[queue->Front].IDE;
    txmessage.Identifier = queue->FDCAN_DataSend[queue->Front].ID;
    txmessage.DataLength = queue->FDCAN_DataSend[queue->Front].DLC;

    memcpy(Tx_msg, queue->FDCAN_DataSend[queue->Front].Data, txmessage.DataLength * sizeof(uint8_t));
    queue->Front = (queue->Front + 1) % FDCAN_QUEUESIZE;

    if (queue->Canx == FDCAN1)
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txmessage, Tx_msg);
    else if (queue->Canx == FDCAN2)
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txmessage, Tx_msg);
}
bool CAN_DequeueRx(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef *Rxheader, uint8_t *Rxdata)
{
    if (CAN_Queue_IfEmpty(queue))
        return 0;
    Rxheader->IdType = queue->FDCAN_DataSend[queue->Front].IDE;
    Rxheader->Identifier = queue->FDCAN_DataSend[queue->Front].ID;
    Rxheader->RxFrameType = FDCAN_DATA_FRAME;
    Rxheader->DataLength = queue->FDCAN_DataSend[queue->Front].DLC;
    memcpy(Rxdata, queue->FDCAN_DataSend[queue->Front].Data, sizeof(uint8_t) * Rxheader->DataLength);
    queue->Front++;
    queue->Front %= FDCAN_QUEUESIZE;
    return 1;
}
void CAN_Enqueue(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef Rxheader, uint8_t Rxdata[])
{
    if (CAN_Queue_IfFull(queue))
        return;
    queue->FDCAN_DataSend[queue->Rear].DLC = Rxheader.DataLength;
    queue->FDCAN_DataSend[queue->Rear].ID = Rxheader.Identifier;
    queue->FDCAN_DataSend[queue->Rear].IDE = Rxheader.IdType;
    memcpy(queue->FDCAN_DataSend[queue->Rear].Data, Rxdata, sizeof(uint8_t) * Rxheader.DataLength);
    queue->Rear++;
    queue->Rear %= FDCAN_QUEUESIZE;
}
void HeaderPrepare(uint32_t sendCode, uint32_t datalen, FDCAN_RxHeaderTypeDef *rxheader)
{
    rxheader->IdType = FDCAN_EXTENDED_ID;
    rxheader->Identifier = sendCode;
    rxheader->DataLength = datalen;
}
