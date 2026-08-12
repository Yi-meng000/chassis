#ifndef __WAVEFORM_H__
#define __WAVEFORM_H__

#include "includes.h"
#include "vector.h"
#include "usart.h"

#define VOFA_ENABLE 1

#define VOFA_FLOAT_NUM 6

// 调试数据的后缀
#define VOFA_SUFFIX1 0x00
#define VOFA_SUFFIX2 0x00
#define VOFA_SUFFIX3 0x80
#define VOFA_SUFFIX4 0x7F

// 调试数据包结构体
typedef struct _vofa_msg
{
    float datach[VOFA_FLOAT_NUM];
    u8 tail[4];
} VOFATxMsgTypedef;

// 外部变量声明
extern VOFATxMsgTypedef VofaTxPack;

// 函数声明
void VOFA_InitTxMsg(VOFATxMsgTypedef *txmsgpack);
void VOFA_DealTxMsg(VOFATxMsgTypedef *txmsgpack);

#endif /* __WAVEFORM_H__ */
