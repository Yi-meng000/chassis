#include "Cameracom.h"

CameraMsgTypedef CameraMsg;          // 调试器消息
CameraRxMsgPackTypedef CameraRxPack; // 接收数据包
CameraLightMsg CameraLightPack;
 u8 CameraLight = 0;
bool Camera_NormalizeRxMsg(CameraMsgTypedef *CameraMsg,
                           CameraRxMsgPackTypedef *rxPack)
{
    u16 rxIndex = 0;

#if CAMERA_RX_BOOL_NUM
    u8 boolBit = 0;
    for (u8 i = 0; i < CAMERA_RX_BOOL_NUM; i++)
    {
        rxPack->Bools[i] =
            (CameraMsg->RxData[rxIndex] & (0x01 << boolBit++)) ? true : false;

        if (8 <= boolBit)
        {
            boolBit = 0;
            rxIndex++;
        }
    }
    rxIndex++;
    #endif
    #if CAMERA_RX_BYTE_NUM
        for (u8 i = 0; i < CAMERA_RX_BYTE_NUM; i++)
        {
            rxPack->Bytes[i] = CameraMsg->RxData[rxIndex++];
        }
        // if(rxPack->Bytes[0] == 4 || rxPack->Bytes[0] == )
    #endif
    #if CAMERA_RX_SHORT_NUM
        for (u8 i = 0; i < CAMERA_RX_SHORT_NUM; i++)
        {
            rxPack->Shorts[i] = MSG_Byte2Int16(CameraMsg->RxData, rxIndex);
            rxIndex += 2;
        }
    #endif

    #if CAMERA_RX_INT_NUM
        for (u8 i = 0; i < CAMERA_RX_INT_NUM; i++)
        {
            rxPack->Ints[i] = Byte2Int32(CameraMsg->RxData, rxIndex);
            rxIndex += 4;
        }
    #endif
    #if CAMERA_RX_FLOAT_NUM
        for (int i = 0; i < CAMERA_RX_FLOAT_NUM; i++)
        {
            memcpy(rxPack->Floats + i, CameraMsg->RxData + rxIndex, sizeof(float));
            rxIndex += 4;
        }
    #endif
        if (CAMERA_RX_PACK_SIZE != rxIndex)
        {
            return 0;
        }
        else
        {
            return 1;
        }
}

bool Camera_ReceiveHandler(CameraMsgTypedef *CameraMsg, uint8_t data)
{
    if ((CameraMsg->CameraState == CAMERA_STATE_WAIT_SUFFIX))
    {
        CameraMsg->Suffix = data;
        if (CameraMsg->Suffix == CAMERA_SUFFIX_2)
        {
            CameraMsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
            if (!Camera_NormalizeRxMsg(CameraMsg, &CameraRxPack))
            {
				CameraMsg->RxDataSize = 0;
                return false;
            }
				CameraMsg->RxDataSize = 0;
        }
        else
        {
            CameraMsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
            CameraMsg->RxDataSize = 0;
        }
    }
    else
    {
        if ((CAMERA_STATE_RECEIVING_DATA == CameraMsg->CameraState))
        {
            CameraMsg->Suffix = data;
            if (CAMERA_SUFFIX_1 == CameraMsg->Suffix)
            {
                CameraMsg->CameraState = CAMERA_STATE_WAIT_SUFFIX;
            }
            else
            {
                CameraMsg->RxData[CameraMsg->RxDataSize++] = data;
            }
        }
        else
        {
            if (CAMERA_STATE_RECEIVING_DONE == CameraMsg->CameraState)
            {
                CameraMsg->Prefix = data;
                if (CameraMsg->Prefix == CAMERA_PREFIX_1)
                {
                    CameraMsg->CameraState = CAMERA_STATE_WAIT_PREFIX;
                }
            }
            else
            {
                if (CameraMsg->CameraState == CAMERA_STATE_WAIT_PREFIX)
                {
                    CameraMsg->Prefix = data;
                    if (CameraMsg->Prefix == CAMERA_PREFIX_2)
                    {
                        CameraMsg->CameraState = CAMERA_STATE_RECEIVING_DATA;
                    }
                }
                else
                {
                    CameraMsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
                    CameraMsg->RxDataSize = 0;
                }
            }
            if (CameraMsg->RxDataSize >= CAMERA_DATA_LEN)
            {
                CameraMsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
                CameraMsg->RxDataSize = 0;
            }
        }
    }
    return true;
}
void Camera_LightReceiveHandler(CameraLightMsg *cmsg,u8 data)
{
   if ((cmsg->CameraState == CAMERA_STATE_WAIT_SUFFIX))
    {
        cmsg->Suffix = data;
        if (cmsg->Suffix == LIGHT_SUFFIX_2)
        {
            cmsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
						cmsg->LightState = cmsg->rxdata[0];
							cmsg->RxDataSize = 0;
        }
        else
        {
            cmsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
            cmsg->RxDataSize = 0;
        }
    }
    else
    {
        if ((CAMERA_STATE_RECEIVING_DATA == cmsg->CameraState))
        {
            cmsg->Suffix = data;
            if (LIGHT_SUFFIX_1 == cmsg->Suffix)
            {
                cmsg->CameraState = CAMERA_STATE_WAIT_SUFFIX;
            }
            else
            {
                cmsg->rxdata[cmsg->RxDataSize++] = data;
            }
        }
        else
        {
            if (CAMERA_STATE_RECEIVING_DONE == cmsg->CameraState)
            {
                cmsg->Prefix = data;
                if (cmsg->Prefix == LIGHT_PREFIX_1)
                {
                    cmsg->CameraState = CAMERA_STATE_WAIT_PREFIX;
                }
            }
            else
            {
                if (cmsg->CameraState == CAMERA_STATE_WAIT_PREFIX)
                {
                    cmsg->Prefix = data;
                    if (cmsg->Prefix == LIGHT_PREFIX_2)
                    {
                        cmsg->CameraState = CAMERA_STATE_RECEIVING_DATA;
                    }
                }
                else
                {
                    cmsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
                    cmsg->RxDataSize = 0;
                }
            }
            if (cmsg->RxDataSize > CAMERA_LIGHT_DATA_LEN)
            {
                cmsg->CameraState = CAMERA_STATE_RECEIVING_DONE;
                cmsg->RxDataSize = 0;
            }
        }
    }
    return;
}
void Camera_LightTransmit(void)
{
    u8 index = 0;
    CameraLightPack.txdata[index++] = LIGHT_PREFIX_1;
    CameraLightPack.txdata[index++] = LIGHT_PREFIX_2;

    CameraLightPack.txdata[index++] = 0x02;

    CameraLightPack.txdata[index++] = LIGHT_SUFFIX_1;
    CameraLightPack.txdata[index++] = LIGHT_SUFFIX_2;

    HAL_UART_Transmit_IT(&huart6,CameraLightPack.txdata,index);
}
