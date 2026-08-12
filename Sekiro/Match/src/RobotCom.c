#include "RobotCom.h"
#include "ActcatorCom.h"
#include "Match.h"
RobotComMsgTypedef RobotComMsg;
RobotComRxMsgStruct RobotRxmsg;

bool RobotCom_ReceiveHandler(RobotComMsgTypedef *rmsg, uint8_t data)
{
    if ((rmsg->RobotComState == ROBOTCOM_STATE_WAIT_SUFFIX))
    {
        rmsg->Suffix = data;
        if (rmsg->Suffix == ROBOTCOM_SUFFIX_2)
        {
            rmsg->RobotComState = ROBOTCOM_STATE_RECEIVING_DONE;
            if (!RobotCom_NormalizeRxMsg(rmsg, &RobotRxmsg))
            {
			   rmsg->RxDataSize = 0;
               return false;
            }
            rmsg->RxDataSize = 0;
        }
        else
        {
            rmsg->RobotComState = ROBOTCOM_STATE_RECEIVING_DONE;
            rmsg->RxDataSize = 0;
        }
    }
    else
    {
        if ((ROBOTCOM_STATE_RECEIVING_DATA == rmsg->RobotComState))
        {
            rmsg->Suffix = data;
            if (ROBOTCOM_SUFFIX_1 == rmsg->Suffix)
            {
                rmsg->RobotComState = ROBOTCOM_STATE_WAIT_SUFFIX;
            }
            else
            {
                rmsg->RxData[rmsg->RxDataSize++] = data;
            }
        }
        else
        {
            if (ROBOTCOM_STATE_RECEIVING_DONE == rmsg->RobotComState)
            {
                rmsg->Prefix = data;
                if (rmsg->Prefix == ROBOTCOM_PREFIX_1)
                {
                    rmsg->RobotComState = ROBOTCOM_STATE_WAIT_PREFIX;
                }
            }
            else
            {
                if (rmsg->RobotComState == ROBOTCOM_STATE_WAIT_PREFIX)
                {
                    rmsg->Prefix = data;
                    if (rmsg->Prefix == ROBOTCOM_PREFIX_2)
                    {
                        rmsg->RobotComState = ROBOTCOM_STATE_RECEIVING_DATA;
                    }
                }
                else
                {
                    rmsg->RobotComState = ROBOTCOM_STATE_RECEIVING_DONE;
                    rmsg->RxDataSize = 0;
                }
            }
            if (rmsg->RxDataSize >= ROBOTCOM_DATA_LEN)
            {
                rmsg->RobotComState = ROBOTCOM_STATE_RECEIVING_DONE;
                rmsg->RxDataSize = 0;
            }
        }
    }
    return true;
}

bool RobotCom_NormalizeRxMsg(RobotComMsgTypedef *rmsg, RobotComRxMsgStruct *rxPack)
{
    u8 index = 0;
    rxPack->type = rmsg->RxData[0];
    if (rxPack->type == Common)
    {
        memcpy(&rxPack->payload_len, rmsg->RxData + 1, sizeof(u16));
        if (rxPack->payload_len != rmsg->RxDataSize - 3)
            return false;
        rxPack->path_len = rmsg->RxData[3];
        Sekiro.offpath.rear = rxPack->path_len;
        for (int i = 0; i < rxPack->path_len; i++)
        {
            Sekiro.offpath.path[i] = rmsg->RxData[4 + i];
        }
        Sekiro.offpath.path[rxPack->path_len] = 13;
        index = 4 + rxPack->path_len;
        rxPack->grab_num = rmsg->RxData[index++];
        Sekiro.offpath.side_pick = rxPack->grab_num;
        for (int i = 0; i < rxPack->grab_num; i++)
        {
            Sekiro.offpath.pick_carPath[i] = rmsg->RxData[index + i * 2];
            Sekiro.offpath.KFS_Pos[i] = rmsg->RxData[index + 1 + i * 2];
        }
        // if (Sekiro.offpath.side_pick && Sekiro.offpath.KFS_Pos[0] == Sekiro.offpath.path[0])
        // {
        //     Sekiro.offpath.side_pick--;
        //     if (Sekiro.offpath.side_pick)
        //     {
        //         Sekiro.offpath.KFS_Pos[0] = Sekiro.offpath.KFS_Pos[1];
        //         Sekiro.offpath.pick_carPath[0] = Sekiro.offpath.pick_carPath[1];
        //     }
        // }
        index += rxPack->grab_num * 2;

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
            {
                int src_index = i * 3 + j;
                int dst_index = (3 - i) * 3 + j;
                Sekiro.Map_status.field[dst_index].field_KFS = (KFS_TYPE)rmsg->RxData[index + src_index];
            }

        index += 12;
        if(rmsg->RxData[index] == 2)
        {
            Actparam.warhead_docked--;
            Actparam.warhead_num[0] += 1;
        }
        Sekiro.Docked = (bool)rmsg->RxData[index++];
        if (Sekiro.UpClaw != rmsg->RxData[index])
        {
            Sekiro.UpClaw = rmsg->RxData[index];
            Actuator_UpClaw(Sekiro.UpClaw);
        }
        KFSDiscardDicide(&Sekiro);
        
        RobotCom_ToActuator(rxPack);
        Sekiro.retry_path = Sekiro.offpath;
        return true;
    }
    else if (rxPack->type == QRCodeR1_dis)
    {
        memcpy(&rxPack->ToR1_x, rmsg->RxData + 1, sizeof(s16));
        rxPack->ToR1_x /= 10;
        memcpy(&rxPack->ToR1_y, rmsg->RxData + 3, sizeof(s16));
        rxPack->ToR1_y /= 10;
        memcpy(&rxPack->ToR1Angle, rmsg->RxData + 5, sizeof(float)); // TODO 在串口三 对应有包类型 为4 包头包尾与路径的一致
        rxPack->pos_receive = true;

    }
    return false;
}
void RobotCom_QrcodeScan(RobotComRxMsgStruct *rxPack, u8 data[])
{
    if (data[14] == 0x0D && rxPack->receive_effect)
    {
        Sekiro.Docked = (bool)(data[12] - 0x30);
        Sekiro.R1PlaceKFS = (u8)(data[13] - 0x30);
    }
}
void RobotCom_ToActuator(RobotComRxMsgStruct *rxPack)
{
    FDCAN_RxHeaderTypeDef rxheader = {0};
    u8 data[8] = {0};
    u8 cnt = 0;
    for (int i = 0; i < rxPack->path_len; i++)
    {
        // 处理前3个台阶存在需要静取的情况
        if (Sekiro.offpath.side_pick)
        {
            if (i == 0)
            {
                if (Sekiro.offpath.KFS_Pos[0] == 1 || Sekiro.offpath.KFS_Pos[0] == 3) // 侧静取+400
                {
                    data[cnt++] = Sekiro.offpath.KFS_Pos[0] << 4 | 2;
                }
                else if (Sekiro.offpath.KFS_Pos[0] == 2) // 侧静取+200
                    data[cnt++] = Sekiro.offpath.KFS_Pos[0] << 4 | 1;
            }
        }

        // 非侧取动作,路径下个位置有R2_KFS
        if (Sekiro.Map_status.field[Sekiro.offpath.path[i] - 1].field_KFS == R2_KFS)
        {
            // 处理前3个位置的R2_KFS通用的情况,也只有这里存在+400
            if (i == 0)
            {
                if (Sekiro.offpath.path[0] == 1 || Sekiro.offpath.path[0] == 3)
                {
                    // 丢+400
                    if (Sekiro.Map_status.field[Sekiro.offpath.path[i] - 1].R2_KFSDiscard)
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 5;
                    else // 动取+400
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 4;
                }
                else if (Sekiro.offpath.path[0] == 2)
                {
                    // 丢+200
                    if (Sekiro.Map_status.field[Sekiro.offpath.path[i] - 1].R2_KFSDiscard)
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 6;
                    else // 动取+200
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 3;
                }
            }
            else // 其它位置情况
            {
                if (Sekiro.Map_status.field[Sekiro.offpath.path[i] - 1].height > Sekiro.Map_status.field[Sekiro.offpath.path[i - 1] - 1].height)
                {
                    // 丢+200
                    if (Sekiro.Map_status.field[Sekiro.offpath.path[i] - 1].R2_KFSDiscard)
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 6;
                    else // 动取+200
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 3;
                }
                else
                {
                    // 丢-200
                    if (Sekiro.Map_status.field[Sekiro.offpath.path[i] - 1].R2_KFSDiscard)
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 9;
                    else // 动取-200
                        data[cnt++] = Sekiro.offpath.path[i] << 4 | 8;
                }
            }
        }
        else // 下个位置没有R2_KFS
        {
            data[cnt++] = Sekiro.offpath.path[i] << 4 | 0;
        }

        // 通常的侧取情况
        if (Sekiro.offpath.side_pick)
        {
            if (i == 0)
            {
                // 前三个的位置要侧取两个
                if (Sekiro.offpath.KFS_Pos[1] == 1 || Sekiro.offpath.KFS_Pos[1] == 3 && Sekiro.offpath.side_pick > 1)
                {
                    if (Sekiro.offpath.path[0] == 2)
                        data[cnt++] = Sekiro.offpath.path[i] + 3 << 4 | 1;
                }
                else if (Sekiro.offpath.KFS_Pos[1] == 2 && Sekiro.offpath.side_pick > 1)
                {
                    data[cnt++] = Sekiro.offpath.path[i] + 3 << 4 | 7;
                }
            }
            else if (Sekiro.offpath.path[i] == Sekiro.offpath.pick_carPath[0] || (Sekiro.offpath.path[i] == Sekiro.offpath.pick_carPath[1] && Sekiro.offpath.side_pick > 1)) // 需要触发进行侧取的位置
            {
                if (Sekiro.offpath.path[i] == Sekiro.offpath.pick_carPath[0])
                {
                    if (Sekiro.Map_status.field[Sekiro.offpath.pick_carPath[0] - 1].height < Sekiro.Map_status.field[Sekiro.offpath.KFS_Pos[0] - 1].height) // 侧静取+200
                        data[cnt++] = Sekiro.offpath.path[i] + 3 << 4 | 1;
                    else // 侧静取-200
                        data[cnt++] = Sekiro.offpath.path[i] + 3 << 4 | 7;
                }

                if (Sekiro.offpath.path[i] == Sekiro.offpath.pick_carPath[1] && Sekiro.offpath.side_pick > 1)
                {
                    if (Sekiro.Map_status.field[Sekiro.offpath.pick_carPath[1] - 1].height < Sekiro.Map_status.field[Sekiro.offpath.KFS_Pos[1] - 1].height)
                        data[cnt++] = Sekiro.offpath.path[i] + 3 << 4 | 1;
                    else
                        data[cnt++] = Sekiro.offpath.path[i] + 3 << 4 | 7;
                }
            }
        }
    }
    HeaderPrepare(MASTER_ACTUATOR_MAPSTATUS, cnt, &rxheader);
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}

bool RobotCom_TraceOnReceive(MatchTraceOn *trace,u8 data)
{
    if(trace->RobotComState == ROBOTCOM_STATE_WAIT_SUFFIX)
    {
        if(data == TraceOn_Suffix)
        {
            if(++trace->suffix_cnt >=3)
            {
                trace->RobotComState = ROBOTCOM_STATE_RECEIVING_DONE;
                trace->suffix_cnt = 0;
                if(RobotCom_TraceRxMsgHandler(trace))
                {
                    trace->RxDataSize = 0;
                    return true;
                }
                else
                {
                    trace->RxDataSize = 0;
                    return false;
                }
            }
        }
    }
    else if(trace->RobotComState == ROBOTCOM_STATE_RECEIVING_DATA)
    {
        if(trace->RxDataSize > 3)
        {
            trace->RobotComState = ROBOTCOM_STATE_RECEIVING_DONE;
            trace->RxDataSize = 0;
            return false;
        }
        if(data == TraceOn_Suffix)
        {
            trace->suffix_cnt++;
            trace->RobotComState = ROBOTCOM_STATE_WAIT_SUFFIX;
        }
        else
            trace->rxdata[trace->RxDataSize++] = data;
    }
    else if(trace->RobotComState == ROBOTCOM_STATE_RECEIVING_DONE)
    {
        if(data == TraceOn_Prefix)
            trace->RobotComState = ROBOTCOM_STATE_RECEIVING_DATA;
    }
		return 0;
}
bool RobotCom_TraceRxMsgHandler(MatchTraceOn *trace)
{
    if(trace->rxdata[0] == 0)
    {
        switch (trace->rxdata[1])
        {
        case 1:
            ChassisEnable(1);
						Chassis.Enable = true;
						Actparam.enable = true;
            Actuator_DMSetZero();
            osDelay(500);
            Actuator_Enable(&Actparam,1);
            break;
        case 2:
            Sekiro.MatchStart = true;
            Sekiro.automatic = true;
            break;
        case 3:
            Sekiro.MatchStart = false;
            Sekiro.automatic = false;
            if(Sekiro.current_zone == ZONE2 || Actparam.warhead_docked == WarheadFetch_Num)
                Sekiro.Zone2Restart = true;
            else if(Sekiro.current_zone == ZONE3)
                Sekiro.Zone3Restart = true;
            MatchPhase = MatchInit;
            osDelay(500);
            Actuator_Reset(&Actparam);
            Chassis.Enable = false;
            Actparam.enable = false;
            sendChassisReset();
            Chassis.IsRunningTraj = false;
            Chassis.LockPoint = false;
            Chassis.climbover = false;
						
            break;
        case 4:
            Sekiro.side = trace->rxdata[1];
            MFCrossPos_Init(&Sekiro);
            break;
        default:
            break;
        }
        return true;
    }
    return false;
}



