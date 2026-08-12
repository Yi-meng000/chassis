#ifndef __FD_CANQUEUE_H__
#define __FD_CANQUEUE_H__

#include "fdcan.h"
#include "includes.h"

#define FDCAN_QUEUESIZE 50

typedef struct
{
    uint32_t ID;
    uint8_t DLC;
    uint32_t IDE;
    uint8_t Data[8];
    bool InConGrpFlag;
} FDCAN_DataStruct;

typedef struct
{
    uint8_t Front, Rear;
    FDCAN_GlobalTypeDef *Canx;
    FDCAN_DataStruct FDCAN_DataSend[FDCAN_QUEUESIZE];
} FDCAN_SendQueueType;

typedef struct
{
    int32_t SendNumber;
    int32_t ReceiveNumber;
    uint32_t TimeOut;
    int32_t SendSum;
    FDCAN_SendQueueType SendQueue;
} MsgControlGrpType;

void CAN_InitSendQueue(void);
bool CAN_Queue_IfEmpty(FDCAN_SendQueueType *queue);
bool CAN_Queue_IfFull(FDCAN_SendQueueType *queue);
void CAN_DequeueTx(FDCAN_SendQueueType *queue);
bool CAN_DequeueRx(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef *Rxheader, uint8_t *Rxdata);
void CAN_Enqueue(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef Rxheader, uint8_t Rxdata[]);
void HeaderPrepare(uint32_t sendCode, uint32_t datalen, FDCAN_RxHeaderTypeDef *rxheader);
extern bool Can1FullFlag;
extern bool Can2FullFlag;
extern FDCAN_SendQueueType CAN1_Txqueue, CAN1_Rxqueue, CAN2_Txqueue, CAN2_Rxqueue;

#endif /* __FD_CANQUEUE_H__ */
