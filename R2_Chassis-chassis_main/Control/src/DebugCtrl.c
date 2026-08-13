#include "DebugCtrl.h"

DebugMsgTypedef DebugMsg = {0};           // 调试器消息
DebugRxMsgPackTypedef DebugRxPack = {0};  // 接收数据包
DebugTxMsgPackTypedef DebugTxPack = {0};  // 发送数据包

float testangle = {0};
Bpoint testPoint = {0};
void Debug_Receive(DebugMsgTypedef * debugMsg,uint8_t data)
{
    if(debugMsg->GetPrefix && !debugMsg->GetSuffix)
    {
        debugMsg->Suffix = data;
        if(debugMsg->Suffix == DEBUG_SUFFIX)
        {
            uint8_t sum = 0;
            for(int i = 0 ; i < debugMsg->RxDataSize - 1 ;i++)
                sum += debugMsg->RxData[i];
            debugMsg->GetPrefix = 0;
            debugMsg->GetSuffix = true;
            if(sum == debugMsg->RxData[debugMsg->RxDataSize - 1])
            {
                if(!Debug_NormalizeRxMsg(debugMsg,&DebugRxPack))
                  return;
            }
            debugMsg->RxDataSize = 0;
            debugMsg->GetPrefix = 0;
        }
        else
            debugMsg->RxData[debugMsg->RxDataSize++] = data;
    }
    else
    {
        debugMsg->Prefix = data;
        if(debugMsg->Prefix == DEBUG_PREFIX)
        {
            debugMsg->GetPrefix = 1;
        }
        if(debugMsg->RxDataSize >= DEBUG_DATA_LEN)
        {
            debugMsg->RxDataSize = 0;
            debugMsg->GetPrefix = 0;
        }
    }
}
bool Debug_NormalizeRxMsg(DebugMsgTypedef* debugMsg,DebugRxMsgPackTypedef* rxPack)
{
    u16 rxIndex = 0;
    #if DEBUG_RX_BOOL_NUM
    uint8_t boolBit = 0;
    for (uint8_t i = 0; i < DEBUG_RX_BOOL_NUM; i++) {
        rxPack->Bools[i] =
            (debugMsg->RxData[rxIndex] & (0x01 << boolBit++)) ? true : false;

        if (8 <= boolBit) {
        boolBit = 0;
        rxIndex++;
        }
    }
    rxIndex++;
    #endif
    #if DEBUG_RX_BYTE_NUM
    for (uint8_t i = 0; i < DEBUG_RX_BYTE_NUM; i++) {
        rxPack->Bytes[i] = debugMsg->RxData[rxIndex++];
    }
    #endif
    #if DEBUG_RX_SHORT_NUM
    for (uint8_t i = 0; i < DEBUG_RX_SHORT_NUM; i++) {
        rxPack->Shorts[i] = MSG_Byte2Int16(debugMsg->RxData, rxIndex);
        rxIndex += 2;
    }
    #endif
    #if DEBUG_RX_INT_NUM
      for (u8 i = 0; i < DEBUG_RX_INT_NUM; i++) {
        rxPack->Ints[i] = MSG_Byte2Int32(debugMsg->RxData, rxIndex);
        rxIndex += 4;
      }
    #endif
    #if DEBUG_RX_FLOAT_NUM
    for (int i = 0; i < DEBUG_RX_FLOAT_NUM; i++) {
        memcpy(rxPack->Floats + i, debugMsg->RxData + rxIndex, sizeof(float));
        rxIndex += 4;
    }
    #endif
    if(rxIndex == RX_PACK_SIZE)
        return 1;
    else
        return 0;
}
/**
 * @brief 发送调试消息
 * @param debugMsg 调试器消息指针
 * @param txPack 发送数据包指针
 * @return Master_StatusTypeDef 操作结果
 */
bool Debug_SendMsg(DebugMsgTypedef* debugMsg,
                                   DebugTxMsgPackTypedef* txPack) {
  u16 txIndex = 0;
  uint8_t sum = 0;

  debugMsg->TxData[txIndex++] = DEBUG_PREFIX;
#if DEBUG_TX_BYTE_NUM
  for (uint8_t i = 0; i < DEBUG_TX_BYTE_NUM; i++) {
    debugMsg->TxData[txIndex++] = txPack->Bytes[i];
  }
#endif
#if DEBUG_TX_SHORT_NUM
  for (uint8_t i = 0; i < DEBUG_TX_SHORT_NUM; i++) {
    MSG_Int162Byte(txPack->Shorts[i], debugMsg->TxData, txIndex);
    txIndex += 2;
  }
#endif
#if DEBUG_TX_INT_NUM
  for (uint8_t i = 0; i < DEBUG_TX_INT_NUM; i++) {
    MSG_Int322Byte(txPack->Ints[i], debugMsg->TxData + txIndex);
    txIndex += 4;
  }
#endif
#if DEBUG_TX_FLOAT_NUM
  for (uint8_t i = 0; i < DEBUG_TX_FLOAT_NUM; i++) {
    memcpy(debugMsg->TxData + txIndex, txPack->Floats + i, sizeof(float));
    txIndex += 4;
  }
#endif
  for (uint8_t i = 1; i < txIndex; i++) {
    sum += debugMsg->TxData[i];
  }

  debugMsg->TxData[txIndex++] = sum;
  debugMsg->TxData[txIndex++] = DEBUG_SUFFIX;
	SCB_CleanDCache_by_Addr((uint32_t *)(debugMsg->TxData),txIndex);
//  if (HAL_UART_Transmit_IT(&huart2, debugMsg->TxData, txIndex) != HAL_OK) {
//    Error_Handler();
//  }
  return 1;
}

bool Debug_ProcessRxMsg(CHASSIS *chassis,DebugRxMsgPackTypedef* rxPack)
{
  if(DebugMsg.GetSuffix)
  {
    float tmpVelx = 0, tmpVely = 0,tmpAngw = 0, tempaimangle = 0;
    uint8_t debugModeID = rxPack->Bytes[0];
//    MatchFlag.matchMode = debugModeID;
   if(rxPack->Bools[4])
   {
       __disable_irq();
       NVIC_SystemReset();
   }
   if(rxPack->Bools[0] && rxPack->Bools[1])
   {
     chassis->Enable = true;
     Motor_Enable(1,chassis);
   }
   else if(!rxPack->Bools[0] && rxPack->Bools[1])
   {
     chassis->Enable = false;
     Motor_Enable(0,chassis);
   }
//    //底盘参数部分
    tempaimangle = (float)(rxPack->Shorts[5]);
    testangle = tempaimangle;
    testPoint.x = rxPack->Shorts[3];
    testPoint.y = rxPack->Shorts[4];
    if(rxPack->Bools[6])
      chassis->Ascend = 1;
    if(rxPack->Bools[7])
      chassis->Descend = 1;

   switch (debugModeID)
   {
   case 0: // manual 
    tmpVelx = (float)(rxPack->Shorts[0]) * CHASSIS_MANUAL_MAX_VELOCITY / 128.f;
    tmpVely = (float)(rxPack->Shorts[1]) * CHASSIS_MANUAL_MAX_VELOCITY / 128.f;
    tmpAngw = (float)(rxPack->Shorts[2]) * CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY / 128.f;
     /* code */
    chassis->ChassisPosSet.vx = tmpVelx;
    chassis->ChassisPosSet.vy = tmpVely;
    chassis->ChassisPosSet.w  = tmpAngw;
    // if(rxPack->Bools[3])
    // {
    //    chassisLockAngle(chassis,(float)rxPack->Shorts[5]);
    // }
		// if(rxPack->Bools[2])
		// {
		// 	crossLock(chassis);
		// }
    // if(rxPack->Bools[5])
    // {
    //     chassis->LockPoint = 1;
    // }
    break;
   case 1:
    //  if(rxPack->Bools[3])
    //  {
    //    chassisLockAngle(chassis,(float)rxPack->Shorts[5]);
    //  }
			// if(rxPack->Bools[5])
			// {
			// 	chassis->LockPoint = 1;
			// }
     break;
   case 2://TODO
       
     break;
   default:
     break;
   }

    DebugMsg.GetSuffix = 0;
    return true;
  }
  return false;
}
void Debug_ProcessTxMsg(CHASSIS *chassis,DebugTxMsgPackTypedef* txPack)
{

 txPack->Shorts[0] = chassis->ChassisPosReal.x;
 txPack->Shorts[1] = chassis->ChassisPosReal.y;
 txPack->Floats[0] = chassis->ChassisPosReal.angle;

  Debug_SendMsg(&DebugMsg,txPack);

}
