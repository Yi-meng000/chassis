#include "ChassisComm.h"
#include "solenoid.h"
#include "MathFunc.h"

void Master_Response(uint8_t DLC, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2)
{
    if (CAN_Queue_IfFull(&CAN1_Txqueue))
    {
        return;
    }
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = ID;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = DLC;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].IDE = FDCAN_EXTENDED_ID;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = Data0;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1] = Data1;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[2] = Data2;
    CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
}
void Motor_Enable(bool enable, CHASSIS *chassis)
{
    for (int i = 0; i < 4; i++)
    {
#if ZMDR_CHASSIS
        Zmotor[i].Enable = enable;
        Zmotor[i].Begin = enable;
        Zmotor[i].mode = enable ? Zdrive_Postion : Zdrive_Disable;

        Zmotor[i + 4].Enable = enable;
        Zmotor[i + 4].Begin = enable;
        Zmotor[i + 4].mode = enable ? Zdrive_Speed : Zdrive_Disable;
        ZdriveSet(0, i + 1, PosIn);

#endif
#if VESC_CHASSIS
        Vescmotor[i].Enable = enable;
        Vescmotor[i].Begin = enable;
        Vescmotor[i].Mode = enable ? VESC_RPM : VescBrake_Mode;
#endif
#if DJ_CHASSIS
        DJmotor[i].Enable = enable;
        DJmotor[i].Begin = enable;
        DJmotor[i].MODE = DJ_RPM;
#endif
    }

    //    for(int i = 8 ; i < 12; i++)
    //    {
    //        Zmotor[i].pvtparam.PVTflag = 1;
    //        ZdriveSet(Zdrive_Hz / 1000.f,i+1,Pos_Vel_TimeGap);
    //        ZdriveSet(2,i+1,Answer_Mode);
    //        ZdriveSet(0,i+1,LIF);
    //    }

    // for (int i = 8; i < 12; i++)
    // {
    //     ZdriveSet(0, i + 1, Pur);
    //     Zmotor[i].Enable = enable;
    //     Zmotor[i].Begin = enable;
    //     Zmotor[i].mode = enable ? Zdrive_Postion : Zdrive_Disable;
    // }

    // Zmotor[12].Enable = enable;
    // Zmotor[12].Begin = enable;
    // Zmotor[12].mode = Zdrive_Speed;
    // Zmotor[13].Enable = enable;
    // Zmotor[13].Begin = enable;
    // Zmotor[13].mode = Zdrive_Speed;
    Master_Response(2, SLAVE_CHASSIS_ENABLE, 'M', enable, 0);
    chassis->Enable = enable;
    ChassisRun = RUN_NORMAL;
}
void Master_CarVel(uint8_t *Rx_data, CHASSIS *chassis)
{
    int16_t tmpVx, tmpVy, tmpVw;
    tmpVx = (int16_t)(Rx_data[1] << 8 | Rx_data[0]);
    tmpVy = (int16_t)(Rx_data[3] << 8 | Rx_data[2]);
    tmpVw = (int16_t)(Rx_data[5] << 8 | Rx_data[4]);
    chassis->ChassisPosSet.vx = (float)(tmpVx / 1000.f);
    chassis->ChassisPosSet.vy = (float)(tmpVy / 1000.f);
    chassis->ChassisPosSet.w = (float)(tmpVw / 100.f);
    if (Rx_data[6] == 'C' && Rx_data[7] == 'B')
    {
        ChassisRun = RUN_CROSSBRAKE;
        // if(SteerTurn_Detect(chassis))
        // {
        //     Master_Response(2,SLAVE_CHASSIS_SET_CARVEL,'C','D',0);
        // }
    }
    else if (Rx_data[6] == 'P' && Rx_data[7] == 'F')
    {
        ChassisRun = RUN_PREFORNEXT;
        if (SteerTurn_Detect(chassis))
        {
            ChassisRun = RUN_NORMAL;
            Master_Response(2, SLAVE_CHASSIS_SET_CARVEL, 'P', 'D', 0);
        }
    }
    else
        ChassisRun = RUN_NORMAL;
    // if(chassis->Enable && !chassis->Ascend && !chassis->Descend)
    //     setCtrlMsg(chassis);
}
/**
 * @brief ??????? ??? CarVelSet(0,0,0,&Chassis);
 *
 * @param vx m/s???
 * @param vy m/s???
 * @param w �/s???
 * @param chassis
 */
void CarVelSet(float vx, float vy, float w, CHASSIS *chassis)
{
    chassis->ChassisPosSet.vx = vx;
    chassis->ChassisPosSet.vy = vy;
}

void Drive_RPM_Answer(CHASSIS *chassis)
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    if (CAN_Queue_IfFull(&CAN1_Txqueue))
        return;
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[FL].DriveMotorValueReal.speed) * 5.f, &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[FR].DriveMotorValueReal.speed) * 5.f, &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[BR].DriveMotorValueReal.speed) * 5.f, &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[BL].DriveMotorValueReal.speed) * 5.f, &index);
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 8;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = SLAVE_CHASSIS_ASK_DRIVE_SPEED;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].IDE = FDCAN_EXTENDED_ID;
    memcpy(CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data, Data, 8);
    CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
}
void Drive_Current_Answer(CHASSIS *chassis)
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    if (CAN_Queue_IfFull(&CAN1_Txqueue))
        return;
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[0].DriveMotorValueReal.current), &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[1].DriveMotorValueReal.current), &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[2].DriveMotorValueReal.current), &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[3].DriveMotorValueReal.current), &index);
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 8;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = SLAVE_CHASSIS_ASK_DRIVE_SPEED;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].IDE = FDCAN_EXTENDED_ID;
    CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
}

void Rotate_Position_Answer(CHASSIS *chassis)
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    float tmpAngle[4] = {0};
    if (CAN_Queue_IfFull(&CAN1_Txqueue))
        return;
    for (int i = 0; i < 4; i++)
    {
        int temp = floor((*chassis->wheel[i].SteerMotorValueReal.angle / 180.f) + 0.5f);
        tmpAngle[i] = (*chassis->wheel[i].SteerMotorValueReal.angle - (180.f * temp)) * 10;
    }
    buffer_append_int16(Data, (int16_t)(tmpAngle[FL]), &index);
    buffer_append_int16(Data, (int16_t)(tmpAngle[FR]), &index);
    buffer_append_int16(Data, (int16_t)(tmpAngle[BL]), &index);
    buffer_append_int16(Data, (int16_t)(tmpAngle[BR]), &index);
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 8;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = SLAVE_CHASSIS_ASK_STEER_ANGLE;
    memcpy(CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data, Data, 8);
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].IDE = FDCAN_EXTENDED_ID;
    CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
}
void Rotate_Current_Answer(CHASSIS *chassis)
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    if (CAN_Queue_IfFull(&CAN1_Txqueue))
        return;
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[0].SteerMotorValueReal.current), &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[1].SteerMotorValueReal.current), &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[2].SteerMotorValueReal.current), &index);
    buffer_append_int16(Data, (int16_t)(*chassis->wheel[3].SteerMotorValueReal.current), &index);
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 8;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = SLAVE_CHASSIS_ASK_STEER_CURRENT;
    CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].IDE = FDCAN_EXTENDED_ID;
    CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
}
void setCtrlMsg(CHASSIS *chassis)
{
    for (int i = 0; i < 4; i++)
    {
#if ZMDR_CHASSIS
        chassis->wheel[i].wheelMotor->valSetNow.speed = chassis->wheel[i].VSet;
#if SteeringWheel
        chassis->wheel[i].steerMotor->valSetNow.angle = chassis->wheel[i].angleSetDeg;
#endif
#endif
#if VESC_CHASSIS
        chassis->wheel[i].wheelMotor->valSet.speed = chassis->wheel->VSet;
#endif
#if DJ_CHASSIS
        chassis->wheel[i].wheelMotor->valSet.speed = (s16)chassis->wheel->VSet;
#endif
    }
}
void Chassis_SelfCheck(CHASSIS *chassis)
{
    bool errorflag = 0;
    for (int i = 0; i < USE_ZDRIVE_NUM; i++)
    {
        if (Zmotor[i].statusflag.err != Zdrive_Well)
        {
            errorflag = true;
            if (Zmotor[i].statusflag.err == Zdrive_LoseEncoder1)
            {
                ZdriveSet(0, i + 1, Err);
                if (i >= 4 && i < 8)
                    ZdriveSet(Zdrive_Speed, i + 1, Mode);
            }
        }
    }
    for (int i = 0; i < USE_DJNUM; i++)
    {
        if (DJmotor[i].statusflag.Overtimeflag || DJmotor[i].statusflag.StuckFlag)
            errorflag = true;
    }
    //    if(errorflag)
    //    {
    //        Master_Response(2,0x020201EE,'C','E',0);
    //    }
    //		else
    //				Master_Response(2,0x020201EE,'C','R',0);
}
void ChassisFunc(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    switch (Rxheader.Identifier)
    {
    case MASTER_CHASSIS_ENABLE: //????
    {
        Motor_Enable(Rx_data[1], &Chassis);
        break;
    }
    case MASTER_CHASSIS_SET_CARVEL:
				if(Chassis.Descend == 0 && Chassis.Ascend == 0)
					Master_CarVel(Rx_data, &Chassis);
        break;
    case MASTER_CHASSIS_ASK_DRIVE_SPEED: //????????
    {
        Drive_RPM_Answer(&Chassis);
        break;
    }
    case MASTER_CHASSIS_ASK_DRIVE_CURRENT: //????????
    {
        Drive_Current_Answer(&Chassis);
        break;
    }
    case MASTER_CHASSIS_ASK_STEER_ANGLE:
    {
        Rotate_Position_Answer(&Chassis);
        break;
    }
    case MASTER_CHASSIS_ASK_STEER_CURRENT:
    {
        Rotate_Current_Answer(&Chassis);
        break;
    }
    case MASTER_CHASSIS_ASCEND:
        Chassis.Ascend = 1;
        ChassisRun = RUN_NORMAL;
        if (Rx_data[1] == 'H')
        {
            box_get = 0;
        }
        else if (Rx_data[1] == 'R')
        {
            box_get = 2;
        }
        else
        {
            box_get = 1;
        }

        if (Rx_data[2] == 'S')
            climb_height = ClimbHeight_200mm;
        else if (Rx_data[2] == 'L')
            climb_height = ClimbHeight_400mm;
        break;
    case MASTER_CHASSIS_DESCEND:
        Chassis.Descend = 1;
        ChassisRun = RUN_NORMAL;
        if (Rx_data[1] == 'H')
            box_get = 0;
        else if (Rx_data[1] == 'R')
        {
            box_get = 2;
        }
        else
            box_get = 1;
        if (Rx_data[2] == 'S')
            climb_height = ClimbHeight_200mm;
        else if (Rx_data[2] == 'L')
            climb_height = ClimbHeight_400mm;
        break;
    case MASTER_CHASSIS_DEBUG_NEXT:
        // DEBUG_CHANGE 启用时，通过主控板命令触发单步进入下一阶段
        if (DEBUG_CHANGE)
        {
            if (Chassis.Ascend)
                Climb_Ascend_DebugNext();
            else if (Chassis.Descend)
                Climb_Descend_DebugNext();
        }
        Master_Response(2, SLAVE_CHASSIS_DEBUG_NEXT, 'O', 'K', 0);
        break;
    case MASTER_CHASSIS_CLIMBW:
        Chassis.ChassisPosSet.w = (int16_t)(Rx_data[1] << 8 | Rx_data[0]) / 100.f;
        Chassis.ChassisPosReal.x = (int16_t)(Rx_data[3] << 8 | Rx_data[2]);
        Chassis.ChassisPosReal.y = (int16_t)(float)(Rx_data[5] << 8 | Rx_data[4]);
        break;
    case MASTER_CHASSIS_RESET: //????
    {
        __disable_irq();
        NVIC_SystemReset();
        break;
    }
    case MASTER_BROADCAST_CHECK:
    {
        Chassis_SelfCheck(&Chassis);
        break;
    }
    case MASTER_CHASSIS_PUSHUP:
        switch (Rx_data[1])
        {
        case 'Z':
            Climb_SetHeight(ClimbHeight_Zero);
            break;
        case 'H':
            Climb_SetHeight(ClimbHeight_400mm);
            break;
        case 'L':
            Climb_SetHeight(ClimbHeight_200mm);
            up_200mm_get_box = 1;
            break;
        default:
            break;
        }
        ChassisRun = RUN_NORMAL;
        Chassis.StandUp = 1;
        break;
    case MASTER_CHASSIS_BETHEONE:
        ChassisRun = RUN_NORMAL;
        Chassis.ClimbUp2R1 = 1;
        break;
    case MASTER_CHASSIS_BACKDOWN:
        ChassisRun = RUN_NORMAL;
				Chassis.ClimbBackDown = 1;
        Master_Response(2, SLAVE_CHASSIS_BACKDOWN, 'O', 'K', 0);
        break;
    default:
        break;
    }
}
