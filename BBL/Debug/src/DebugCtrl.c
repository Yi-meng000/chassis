#include "DebugCtrl.h"
#include "Match.h"
DebugMsgTypedef DebugMsg = {0};          // 调试器消息
DebugRxMsgPackTypedef DebugRxPack = {0}; // 接收数据包
DebugTxMsgPackTypedef DebugTxPack = {0}; // 发送数据包
Bpoint testPoint = {0};
float testangle = 0;
uint8_t testThrehold = 0;
uint8_t testLockPID = 0;
uint16_t CamDepth = 900;
void Debug_Receive(DebugMsgTypedef *debugMsg, uint8_t data)
{
    if (debugMsg->GetPrefix)
    {
        debugMsg->Suffix = data;
        if (debugMsg->Suffix == DEBUG_SUFFIX)
        {
            uint8_t sum = 0;
            for (int i = 0; i < debugMsg->RxDataSize - 1; i++)
                sum += debugMsg->RxData[i];
            debugMsg->GetPrefix = 0;
            debugMsg->GetSuffix = true;
            if (sum == debugMsg->RxData[debugMsg->RxDataSize - 1])
            {
                if (!Debug_NormalizeRxMsg(debugMsg, &DebugRxPack))
                    return;
            }
            debugMsg->RxDataSize = 0;
            debugMsg->GetPrefix = 0;
        }
        else
            debugMsg->RxData[debugMsg->RxDataSize++] = data;
    }
    else if (!debugMsg->GetSuffix)
    {
        debugMsg->Prefix = data;
        if (debugMsg->Prefix == DEBUG_PREFIX)
        {
            debugMsg->GetPrefix = 1;
        }
        if (debugMsg->RxDataSize >= DEBUG_DATA_LEN)
        {
            debugMsg->RxDataSize = 0;
            debugMsg->GetPrefix = 0;
        }
    }
}
bool Debug_NormalizeRxMsg(DebugMsgTypedef *debugMsg, DebugRxMsgPackTypedef *rxPack)
{
    u16 rxIndex = 0;
#if DEBUG_RX_BOOL_NUM
    uint8_t boolBit = 0;
    for (uint8_t i = 0; i < DEBUG_RX_BOOL_NUM; i++)
    {
        rxPack->Bools[i] =
            (debugMsg->RxData[rxIndex] & (0x01 << boolBit++)) ? true : false;

        if (8 <= boolBit)
        {
            boolBit = 0;
            rxIndex++;
        }
    }
    rxIndex++;
#endif
#if DEBUG_RX_BYTE_NUM
    for (uint8_t i = 0; i < DEBUG_RX_BYTE_NUM; i++)
    {
        rxPack->Bytes[i] = debugMsg->RxData[rxIndex++];
    }
#endif
#if DEBUG_RX_SHORT_NUM
    for (uint8_t i = 0; i < DEBUG_RX_SHORT_NUM; i++)
    {
        rxPack->Shorts[i] = MSG_Byte2Int16(debugMsg->RxData, rxIndex);
        rxIndex += 2;
    }
#endif
#if DEBUG_RX_INT_NUM
    for (u8 i = 0; i < DEBUG_RX_INT_NUM; i++)
    {
        rxPack->Ints[i] = MSG_Byte2Int32(debugMsg->RxData, rxIndex);
        rxIndex += 4;
    }
#endif
#if DEBUG_RX_FLOAT_NUM
    for (int i = 0; i < DEBUG_RX_FLOAT_NUM; i++)
    {
        memcpy(rxPack->Floats + i, debugMsg->RxData + rxIndex, sizeof(float));
        rxIndex += 4;
    }
#endif
    if (rxIndex == RX_PACK_SIZE)
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
bool Debug_SendMsg(DebugMsgTypedef *debugMsg,
                   DebugTxMsgPackTypedef *txPack)
{
    u16 txIndex = 0;
    uint8_t sum = 0;

    debugMsg->TxData[txIndex++] = DEBUG_PREFIX;
#if DEBUG_TX_BYTE_NUM
    for (uint8_t i = 0; i < DEBUG_TX_BYTE_NUM; i++)
    {
        debugMsg->TxData[txIndex++] = txPack->Bytes[i];
    }
#endif
#if DEBUG_TX_SHORT_NUM
    for (uint8_t i = 0; i < DEBUG_TX_SHORT_NUM; i++)
    {
        MSG_Int162Byte(txPack->Shorts[i], debugMsg->TxData, txIndex);
        txIndex += 2;
    }
#endif
#if DEBUG_TX_INT_NUM
    for (uint8_t i = 0; i < DEBUG_TX_INT_NUM; i++)
    {
        MSG_Int322Byte(txPack->Ints[i], debugMsg->TxData, txIndex);
        txIndex += 4;
    }
#endif
#if DEBUG_TX_FLOAT_NUM
    for (uint8_t i = 0; i < DEBUG_TX_FLOAT_NUM; i++)
    {
        memcpy(debugMsg->TxData + txIndex, txPack->Floats + i, sizeof(float));
        txIndex += 4;
    }
#endif
    for (uint8_t i = 1; i < txIndex; i++)
    {
        sum += debugMsg->TxData[i];
    }

    debugMsg->TxData[txIndex++] = sum;
    debugMsg->TxData[txIndex++] = DEBUG_SUFFIX;
    SCB_CleanDCache_by_Addr((uint32_t *)(debugMsg->TxData), txIndex);
    if (HAL_UART_Transmit_IT(&huart2, debugMsg->TxData, txIndex) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

bool test_camera_searchR1 = false;
bool Debug_ProcessRxMsg(CHASSIS *chassis, DebugRxMsgPackTypedef *rxPack)
{
    if (DebugMsg.GetSuffix)
    {
        float tmpVelx = 0, tmpVely = 0, tmpAngw = 0;
        // static bool pre_write = 0;
        //  static bool write_tmp = 0;
        //      uint8_t map[12];
        uint8_t debugModeID = rxPack->Bytes[0];
        Actparam.warhead_num[0] = rxPack->Bytes[2];
        if (rxPack->Bools[6])
        {
            switch (rxPack->Bytes[3])
            {
            case 0:
                __disable_irq();
                __HAL_RCC_DMA2_FORCE_RESET();
                __HAL_RCC_DMA2_RELEASE_RESET();
                __HAL_RCC_DMA1_FORCE_RESET();
                __HAL_RCC_DMA1_RELEASE_RESET();
                __HAL_RCC_USART1_FORCE_RESET();
                __HAL_RCC_USART1_RELEASE_RESET();
                __HAL_RCC_USART2_FORCE_RESET();
                __HAL_RCC_USART2_RELEASE_RESET();
                NVIC_SystemReset();
                break;
            case 1:
                sendChassisReset();
                break;
            case 2:
                Actuator_Reset(&Actparam);
                break;
            case 3:
                sendChassisReset();
                Actuator_Reset(&Actparam);
                osDelay(500);
                __disable_irq();
                NVIC_SystemReset();
                break;
            case 4:
                sendChassisReset();
                Actuator_Reset(&Actparam);
                if(Sekiro.current_zone == ZONE2)
                    Sekiro.Zone2Restart = 1;
                else if(Sekiro.current_zone == ZONE3)
                    Sekiro.Zone3Restart = 1;
                MatchPhase = MatchInit;
                Sekiro.MatchStart = 0;
                Sekiro.SkillMatchStart = 0;
                break;
            default:
                break;
            }
        }
        if (rxPack->Bools[0] && rxPack->Bools[1])
        {
            chassis->Enable = true;
            ChassisEnable(1);
        }
        else if (!rxPack->Bools[0] && rxPack->Bools[1])
        {
            chassis->Enable = false;
            ChassisEnable(0);
        }
        if (rxPack->Bools[2] && rxPack->Bools[3])
        {
            Actparam.enable = true;
            Actuator_Enable(&Actparam, 1);
        }
        else if (!rxPack->Bools[2] && rxPack->Bools[3])
        {
            Actparam.enable = false;
            Actuator_Enable(&Actparam, 0);
        }
        // 底盘参数部分

        //  if(rxPack->Bools[23])
        //  {
        //    if(Route_Load(&Sekiro,map))
        //      Route_SendMsg(map,&RouteMsg);
        //  }
        if (rxPack->Bools[23] != Sekiro.side)
        {
            Sekiro.side = rxPack->Bools[23];
            MFCrossPos_Init(&Sekiro);
        }
        //  if(rxPack->Bools[20])
        //  {
        //    Sekiro.Map_status.rear = 0;
        //  }
        if (rxPack->Bools[30])
        {
            Actuator_DMSetZero();
        }

        switch (debugModeID)
        {
        case 0: // manual
            Sekiro.half_auto = 0;
            Sekiro.automatic = 0;
            chassis->sendCorrect_w = 0; // TODO 改这个 置零就没有翻越时纠偏
            tmpVelx = (float)(rxPack->Shorts[0]) * CHASSIS_MANUAL_MAX_VELOCITY / 128.f;
            tmpVely = (float)(rxPack->Shorts[1]) * CHASSIS_MANUAL_MAX_VELOCITY / 128.f;
            tmpAngw = (float)(rxPack->Shorts[2]) * CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY / 128.f;

            testangle = (float)(rxPack->Shorts[5]);
            testPoint.x = rxPack->Shorts[3];
            testPoint.y = rxPack->Shorts[4];

            //		PID_Init(&anglePID,rxPack->Floats[0],rxPack->Floats[1],rxPack->Floats[2],PIDPOS);
            if (rxPack->Bools[18])
            {
                // SensorUsart_Msg.offset_x = testPoint.x - (Sensor_RxPack.Laser_RxPack.floats[0] * 1000.f);
                // SensorUsart_Msg.offset_y = testPoint.y - (Sensor_RxPack.Laser_RxPack.floats[1] * 1000.f);
                // 								SensorUsart_Msg.offset_angle = 90.f - Sensor_RxPack.Laser_RxPack.floats[3];
                //                 //LaserRelocation(&SensorUsart_Msg, &Sensor_TxPack, 0, testPoint, float tmp_y, float tmp_angle)
                LaserRelocation(&SensorUsart_Msg, &Sensor_TxPack, 1, (float)testPoint.x, (float)testPoint.y, testangle);
            }
            chassis->LockPoint = 0;
            chassis->CamLockPoint = 0;
            chassis->IsRunningTraj = 0;
            /* code */
            chassis->ChassisPosSet.vx = tmpVelx;
            chassis->ChassisPosSet.vy = tmpVely;
            chassis->ChassisPosSet.w = tmpAngw;
            if (rxPack->Bools[4])
            {
                chassisLockAngle(chassis, (float)rxPack->Shorts[5]);
            }
            Chassis_carvelSet(chassis);
            chassis->crossBrake = rxPack->Bools[17];

            if (rxPack->Bools[7] && chassis->Enable && !chassis->climbover)
            {
                chassis_Ascend(chassis, rxPack->Bools[21], rxPack->Bytes[4]);
            }
            if (rxPack->Bools[8] && chassis->Enable && !chassis->climbover)
            {
                chassis_Descend(chassis, rxPack->Bools[21], rxPack->Bytes[4]);
            }
            if (rxPack->Bools[25])
            {
                chassis_Upstand(chassis, rxPack->Bytes[5]);
            }
            if (rxPack->Bools[24])
            {
               chassis_Up2R1(chassis);
            }
            //   if(rxPack->Bools[30] != write_tmp)
            //   {
            //     chassis_StepbyStep(chassis,rxPack->Bools[30]);
            //     write_tmp = rxPack->Bools[30];
            //   }

            if (chassis->Enable && chassis->Status != CHASSIS_CLIMBOVER)
            {
                sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f), (s16)(chassis->ChassisPosSet.vy * 1000.f),
                           (s16)(chassis->ChassisPosSet.w * 100.f), (CHASSIS_RUNMODE)chassis->crossBrake);
            }
            // 机构控制
            if (Actparam.enable)
            {
                // if(rxPack->Bools[9] )
                // {
                //     Actuator_WarheadMove(&Actparam,Sekiro.side);
                // }
                if (rxPack->Bools[9] && Actparam.grab == ACT_READY)
                {
                    Actuator_WarheadCatch(&Actparam);
                }
                if (rxPack->Bools[10] && Actparam.grab == ACT_READY)
                {
                    Actuator_WarheadReady(&Actparam, Sekiro.side);
                }
                if (rxPack->Bools[11])
                {
                    Actuator_WarheadRelease(0);
                }
                if (rxPack->Bools[12])
                {
                    Actuator_KFSReady();
                }
                if (rxPack->Bools[15])
                {
                    Actuator_KFSPrepare(&Actparam, rxPack->Bytes[6]);
                }
                if (rxPack->Bools[13] && Actparam.arm == ACT_READY)
                {
                    switch (rxPack->Bytes[4])
                    {
                    case 3:
                        Actuator_KFSCatch(&Actparam, 'H');
                        break;
                    case 4:
                        Actuator_KFSCatch(&Actparam, 'T');
                        break;
                    case 5:
                        Actuator_KFSCatch(&Actparam, 'L');
                        break;
                    case 6:
                        Actuator_KFSCatch(&Actparam, 'P');
                        break;
                    case 7:
                        Actuator_KFSCatch(&Actparam, 'M');
                        break;
                    default:
                        break;
                    }
                }
                if (rxPack->Bools[14] && Actparam.arm == ACT_READY)
                {
                    if (!rxPack->Bools[16])
                        Actuator_KFSPlace(&Actparam, 'M');
                    else if (chassis->ChassisPosReal.z > 1050 || rxPack->Bools[16])
                        Actuator_KFSPlace(&Actparam, 'T');
                }
                if (rxPack->Bools[20] && Actparam.arm == ACT_READY)
                    Actuator_KFSPlace(&Actparam, 'P');
            }
            break;
        case 1:
            Sekiro.half_auto = 0;
            Sekiro.automatic = 0;
            chassis->sendCorrect_w = 0;
            testLockPID = NormalPID;
            LockPointPIDParam[0] = rxPack->Floats[0] / 1000.f;
            LockPointPIDParam[1] = rxPack->Floats[1] / 1000.f;
            LockPointPIDParam[2] = rxPack->Floats[2] / 1000.f;
            // PID_Init(&anglePID, rxPack->Floats[3], rxPack->Floats[4], rxPack->Floats[5], PIDPOS);
            vector2fPIDInit(lockPointPID + NormalPID, LockPointPIDParam, PIDPOS);
            testangle = (float)(rxPack->Shorts[5]);
            testPoint.x = rxPack->Shorts[3];
            testPoint.y = rxPack->Shorts[4];
            CamDepth = rxPack->Shorts[6];
            if (rxPack->Bools[5])
            {
                chassis->LockPoint = 1;
            }
            if (rxPack->Bools[27])
            {
                chassis_Movehorizontal(chassis,1,2);
            }
            //      if(rxPack->Bools[22])
            //      {
            //        Camera_Relocation(&Sekiro,&SensorUsart_Msg,&CameraRxPack,chassis);
            //      } 相机重定位
            // if(rxPack->Bools[19] && !pre_write)
            // {
            // 	MatchMap_Set(rxPack->Bytes[1],&Sekiro);
            // 	pre_write = 1;
            // }
            if (rxPack->Bools[26] && !chassis->IsRunningTraj) // 9750 3900
            {
                vector2d TestTrajPoint[6] = {{0, 0}, {75, 0}, {150, 0}, {225, 0}, {300, 0}, {375, 0}};
                TrajParam_SetPoints(TestTraj, Bezier, 5, TestTrajPoint, 0.8f, -90.f, -90.f, No_Brake);
                BezierParam_Init(Trajhandler + TestTraj, TestTraj, SinF, Spin,
                                0.3f, 0.1f, 0.1f, 0.7f, 0.f, 0.0, 50, 10);
                trajMarker = Zone3_SlopeToEnd;
                chassis->IsRunningTraj = true;
            }
            break;
        case 2: // TODO half_Auto
            Sekiro.half_auto = 1;
            Sekiro.automatic = 0;
            Sekiro.Nxt_move = rxPack->Bools[22];
            chassis->sendCorrect_w = 1; // 半自动里默认纠偏
            if (rxPack->Bools[28])
                chassis->CamLockPoint = true;
            if (rxPack->Bools[9])
            {
                Sekiro.Club_only = true;
            }
            if (rxPack->Bools[11])
            {
                Sekiro.Docked = 1;
            }
            if (rxPack->Bools[14])
            {
                if (rxPack->Bools[16])
                    Sekiro.R1PlaceKFS = 2;
                else
                    Sekiro.R1PlaceKFS = 1;
            }
            //            if (rxPack->Bools[19] && !pre_write)
            //            {
            //                Sekiro.offpath.path[Sekiro.offpath.rear++] = rxPack->Bytes[1];
            //                pre_write = 1;
            //            }
            if (rxPack->Bools[19])
                Sekiro.LeaveZone1 = 0;
            if (rxPack->Bools[7])
            {
                Sekiro.MFcross_only = true;
            }
            if (rxPack->Bools[24])
                Sekiro.Square9_only = true;
            if (rxPack->Bools[31])
            {
                Sekiro.SkillMatchStart = 1;
            }
            break;
        case 3:
            Sekiro.half_auto = 0;
            Sekiro.automatic = 1;
            chassis->sendCorrect_w = 1;
            if (rxPack->Bools[9])
            {
                Sekiro.Club_only = true;
            }
            if (rxPack->Bools[11])
            {
                Sekiro.Docked = 1;
            }
            if (rxPack->Bools[14])
            {
                if (rxPack->Bools[16])
                    Sekiro.R1PlaceKFS = 2;
                else
                    Sekiro.R1PlaceKFS = 1;
            }
            if (rxPack->Bools[7])
            {
                Sekiro.MFcross_only = true;
            }
            //            if (rxPack->Bools[19] && !pre_write)
            //            {
            //                Sekiro.offpath.path[Sekiro.offpath.rear++] = rxPack->Bytes[1];
            //                pre_write = 1;
            //            }
            if (rxPack->Bools[19])
                Sekiro.LeaveZone1 = 1;
            if (rxPack->Bools[24])
                Sekiro.Square9_only = true;
            if (rxPack->Bools[29])
                Sekiro.MatchStart = true;
			if (rxPack->Bools[31])
            {
                Sekiro.SkillMatchStart = 1;
            }
			SkillMatchType = rxPack->Bools[32];
            break;
        case 4:

            break;
        default:
            break;
        }
        DebugMsg.GetSuffix = 0;
        return true;
    }
    return false;
}
void Debug_ProcessTxMsg(CHASSIS *chassis, DebugTxMsgPackTypedef *txPack)
{
    txPack->Bytes[0] = Sekiro.Map_status.rear;
    txPack->Bytes[1] = Actparam.arm;
    txPack->Bytes[2] = Actparam.grab;
    txPack->Bytes[3] = Chassis.Status;
    txPack->Bytes[4] = CameraLightPack.LightState;
    txPack->Bytes[5] = Sekiro.zone2_field;

    txPack->Shorts[0] = chassis->ChassisPosReal.x;
    txPack->Shorts[1] = chassis->ChassisPosReal.y;
    txPack->Shorts[2] = (int16_t)(chassis->ChassisPosSet.vx *1000.f);
    txPack->Shorts[3] = (int16_t)(chassis->ChassisPosSet.vy *1000.f);
    txPack->Shorts[4] = chassis->ChassisPosSet.x;
    txPack->Shorts[5] = chassis->ChassisPosSet.y;
    txPack->Shorts[6] = RobotRxmsg.ToR1_x;
    txPack->Shorts[7] = RobotRxmsg.ToR1_y;

    txPack->Floats[0] = chassis->ChassisPosReal.angle;

    // txPack->Shorts[2] = chassis->wheel[FL].SteerMotorValueReal.angle;
    // txPack->Shorts[3] = chassis->wheel[FR].SteerMotorValueReal.angle;
    // txPack->Shorts[4] = chassis->wheel[BL].SteerMotorValueReal.angle;
    // txPack->Shorts[5] = chassis->wheel[BR].SteerMotorValueReal.angle;

    Debug_SendMsg(&DebugMsg, txPack);
}
