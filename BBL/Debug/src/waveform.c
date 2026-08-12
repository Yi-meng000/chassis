#include "waveform.h"
#include "chassisRun.h"

VOFATxMsgTypedef VofaTxPack = {0};

u8 buffer[(4 * (VOFA_FLOAT_NUM + 1))] = {0};

void VOFA_InitTxMsg(VOFATxMsgTypedef *txmsgpack)
{
    for (u8 i = 0; i < VOFA_FLOAT_NUM; i++)
    {
        txmsgpack->datach[i] = 0;
    }

    txmsgpack->tail[0] = VOFA_SUFFIX1;
    txmsgpack->tail[1] = VOFA_SUFFIX2;
    txmsgpack->tail[2] = VOFA_SUFFIX3;
    txmsgpack->tail[3] = VOFA_SUFFIX4;

    buffer[4 * VOFA_FLOAT_NUM] = txmsgpack->tail[0];
    buffer[4 * VOFA_FLOAT_NUM + 1] = txmsgpack->tail[1];
    buffer[4 * VOFA_FLOAT_NUM + 2] = txmsgpack->tail[2];
    buffer[4 * VOFA_FLOAT_NUM + 3] = txmsgpack->tail[3];
}
void _VOFA_SendMsg(VOFATxMsgTypedef *txmsgpack)
{
    for (u8 i = 0; i < VOFA_FLOAT_NUM; i++)
    {
        MSG_Float2Byte(txmsgpack->datach[i], buffer, 4 * i);
    }
    SCB_CleanDCache_by_Addr((uint32_t *)(buffer), (4 * (VOFA_FLOAT_NUM + 1)));
    HAL_UART_Transmit_IT(&huart9, buffer, (4 * (VOFA_FLOAT_NUM + 1)));
}
void VOFA_DealTxMsg(VOFATxMsgTypedef *txmsgpack)
{
    txmsgpack->datach[0] = (float)(Chassis.ChassisPosSet.vx * 100);
    txmsgpack->datach[1] = (float)(Chassis.ChassisPosSet.vy * 100);
    txmsgpack->datach[2] = (float)(Chassis.ChassisPosSet.w * 100);
    txmsgpack->datach[3] = (float)(Chassis.ChassisPosReal.x);
    txmsgpack->datach[4] = (float)(Chassis.ChassisPosReal.y);
    txmsgpack->datach[5] = (float)(Chassis.ChassisPosReal.angle);

    _VOFA_SendMsg(txmsgpack);
}
