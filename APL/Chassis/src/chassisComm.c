#include "chassisComm.h"
#include "wheelTrain.h"
#include "DebugCtrl.h"
CHASSIS Chassis;
uint8_t Data[8] = {0};
FDCAN_RxHeaderTypeDef Rxheader;

void ChassisEnable(uint8_t enable)
{
    HeaderPrepare(MASTER_CHASSIS_ENABLE, 2, &Rxheader);
    Data[0] = 'M';
    Data[1] = enable;
    Chassis.Status = enable ? CHASSIS_RUN : CHASSIS_DISABLE;
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
}
void sendCarVel(s16 carVx, s16 carVy, s16 carVw, CHASSIS_RUNMODE mode)
{

    HeaderPrepare(MASTER_CHASSIS_SET_CARVEL, 8, &Rxheader);
    memcpy(Data, &carVx, sizeof(s16));
    memcpy(Data + 2, &carVy, sizeof(s16));
    memcpy(Data + 4, &carVw, sizeof(s16));
    switch (mode)
    {
    case RUN_NORMAL:
        memset(Data + 6, 0, sizeof(s16));
        break;
    case RUN_CROSSBRAKE:
        Data[6] = 'C';
        Data[7] = 'B';
        break;
    case RUN_PREFORNEXT:
        Data[6] = 'P';
        Data[7] = 'F';
        break;
    default:
        break;
    }

    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
}
void sendDrivingSpeed(s16 _FL, s16 _FR, s16 _BL, s16 _BR)
{
    HeaderPrepare(MASTER_CHASSIS_SET_DRIVE_SPEED, 8, &Rxheader);
    memcpy(Data, &_FL, sizeof(s16));
    memcpy(Data + 2, &_FR, sizeof(s16));
    memcpy(Data + 4, &_BL, sizeof(s16));
    memcpy(Data + 6, &_BR, sizeof(s16));
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
}

void sendChassisAskMsg(uint32_t askCode)
{
    HeaderPrepare(askCode, 2, &Rxheader);
    switch (askCode)
    {
    case MASTER_CHASSIS_ASK_DRIVE_SPEED:
        Data[0] = 'D';
        Data[1] = 'V';
        break;
    case MASTER_CHASSIS_ASK_DRIVE_CURRENT:
        Data[0] = 'D';
        Data[1] = 'C';
        break;
    case MASTER_CHASSIS_ASK_STEER_ANGLE:
        Data[0] = 'S';
        Data[1] = 'A';
        break;
    case MASTER_CHASSIS_ASK_STEER_CURRENT:
        Data[0] = 'S';
        Data[1] = 'C';
        break;
    default:
        break;
    }
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
}
void sendChassisReset(void)
{
    HeaderPrepare(MASTER_CHASSIS_RESET, 2, &Rxheader);
    Data[0] = 'R';
    Data[1] = 'S';
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}
void chassis_ReceiveHandler(CHASSIS *chassis, FDCAN_RxHeaderTypeDef rxheader, uint8_t Rxdata[])
{
    switch (rxheader.Identifier)
    {
    case SLAVE_CHASSIS_ENABLE:
        chassis->Status = (Rxdata[1] == 1) ? CHASSIS_RUN : CHASSIS_DISABLE;
        break;
    case SLAVE_CHASSIS_SET_CARVEL:
        if (Rxdata[0] == 'P' && Rxdata[1] == 'D')
            chassis->TurnForNextRunResearch = 1;
        break;

    case SLAVE_CHASSIS_DESCEND:
        chassis->Status = CHASSIS_RUN;
        chassis->ChassisPosSet.w = 0;
        chassis->climbover = 0;
        chassis_climbSend(chassis);
        // TODO crossLock(chassis); 必要的话加到状态机里
        break;
    case SLAVE_CHASSIS_ASCEND:
        chassis->Status = CHASSIS_RUN;
        chassis->ChassisPosSet.w = 0;
        chassis->climbover = 0;
        chassis_climbSend(chassis);
        // crossLock(chassis);
        break;
    case SLAVE_CHASSIS_PUSHUP:
        chassis->Status = CHASSIS_RUN;
        chassis->StandUp = 0;
        break;
    case SLAVE_CHASSIS_BETHEONE:
        chassis->Status = CHASSIS_RUN;
        chassis->ClimbUp2R1 = 1;
        break;
    case SLAVE_CHASSIS_ERROR:
        if (Rxdata[1] == 'E' && !chassis->climbover)
        {
            chassis->Status = CHASSIS_ERROR;
        }
        else if (Rxdata[1] == 'R' && !chassis->climbover)
            chassis->Status = CHASSIS_RUN;
        break;
    case SLAVE_CHASSIS_ASK_DRIVE_SPEED:
        chassis->wheel[FL].DriveMotorValueReal.speed = (int16_t)(Rxdata[1] << 8 | Rxdata[0]) / 5.f;
        chassis->wheel[FR].DriveMotorValueReal.speed = (int16_t)(Rxdata[3] << 8 | Rxdata[2]) / 5.f;
        chassis->wheel[BL].DriveMotorValueReal.speed = (int16_t)(Rxdata[5] << 8 | Rxdata[4]) / 5.f;
        chassis->wheel[BR].DriveMotorValueReal.speed = (int16_t)(Rxdata[7] << 8 | Rxdata[6]) / 5.f;
        break;
    case SLAVE_CHASSIS_ASK_STEER_ANGLE:
        chassis->wheel[FL].SteerMotorValueReal.angle = (int16_t)(Rxdata[1] << 8 | Rxdata[0]) / 10.f;
        chassis->wheel[FR].SteerMotorValueReal.angle = (int16_t)(Rxdata[3] << 8 | Rxdata[2]) / 10.f;
        chassis->wheel[BL].SteerMotorValueReal.angle = (int16_t)(Rxdata[5] << 8 | Rxdata[4]) / 10.f;
        chassis->wheel[BR].SteerMotorValueReal.angle = (int16_t)(Rxdata[7] << 8 | Rxdata[6]) / 10.f;
        break;
    default:
        break;
    }
}

void chassis_Ascend(CHASSIS *chassis, bool height, uint8_t grab)
{
    chassis->Status = CHASSIS_CLIMBOVER;
    chassis->climbover = 1;
    chassis->climb_record_x = chassis->ChassisPosReal.x;
    chassis->climb_record_y = chassis->ChassisPosReal.y;
    chassis->climb_angle    = testangle;
    HeaderPrepare(MASTER_CHASSIS_ASCEND, 3, &Rxheader);
    Data[0] = 'A';
    switch (grab)
    {
    case 0:
        Data[1] = 'H';
        break;
    case 1:
        Data[1] = 'G';
        break;
    case 2:
        Data[1] = 'D';
        break;
    case 3:
        Data[1] = 'R';
        break;
    default:
        Data[1] = 'H';
        break;
    }
    Data[2] = height ? 'L' : 'S';
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}
void chassis_Descend(CHASSIS *chassis, bool height, uint8_t grab)
{
    chassis->Status = CHASSIS_CLIMBOVER;
    chassis->climbover = 1;
    chassis->climb_record_x = chassis->ChassisPosReal.x;
    chassis->climb_record_y = chassis->ChassisPosReal.y;
    chassis->climb_angle    = testangle;
    HeaderPrepare(MASTER_CHASSIS_DESCEND, 3, &Rxheader);
    Data[0] = 'D';
    switch (grab)
    {
    case 0:
        Data[1] = 'H';
        break;
    case 1:
        Data[1] = 'G';
        break;
    case 2:
        Data[1] = 'D';
        break;
    case 4:
        Data[1] = 'R';
        break;
    default:
        Data[1] = 'H';
        break;
    }
    Data[2] = height ? 'L' : 'S';
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}

void chassis_SensordataHandle(CHASSIS *chassis, SENSOR_RXPACK *rxPack, SENSORUSART_MSG *msg)
{
//    static u8 re_tmp = 0;
    vector2d tmp, data;
    LaserUsartDeal(rxPack);
    chassis->ChassisPosReal.angle = (rxPack->Laser_RxPack.floats[3] + msg->offset_angle);
    tmp.x = (rxPack->Laser_RxPack.floats[0] * 1000.f) + (float)msg->offset_x;
    tmp.y = (rxPack->Laser_RxPack.floats[1] * 1000.f) + (float)msg->offset_y;
    Filter_Func(&Radar_Filter, tmp, &data);
    chassis->ChassisPosReal.x = roundf(data.x * cosf(DEG2RAD(-msg->offset_angle)) + data.y * sinf(DEG2RAD(-msg->offset_angle)));
    chassis->ChassisPosReal.y = roundf(data.y * cosf(DEG2RAD(-msg->offset_angle)) - data.x * sinf(DEG2RAD(-msg->offset_angle)));
    chassis->ChassisPosReal.z = (int16_t)(rxPack->Laser_RxPack.floats[2] * 1000.f);
    chassis->pitch = rxPack->Laser_RxPack.floats[4];
    msg->getSuffix = 0;
}
void chassis_climbSend(CHASSIS *chassis)
{
    HeaderPrepare(MASTER_CHASSIS_CLIMBW, 8, &Rxheader);

    int16_t tmp_w = (s16)(chassis->ChassisPosSet.w * 100.f);
    int16_t tmp_x = (chassis->ChassisPosReal.x - chassis->climb_record_x);
    int16_t tmp_y = (chassis->ChassisPosReal.y - chassis->climb_record_y);
    if(!chassis->climbover)
    {
        tmp_w = 0;
        tmp_x = 0;
        tmp_y = 0;
    }
    float AngleRad = DEG2RAD(chassis->climb_angle);
    float send_x = (tmp_x * cosf(AngleRad) + tmp_y * sinf(AngleRad));
    float send_y = (tmp_y * cosf(AngleRad) - tmp_x * sinf(AngleRad));
    float output = (int16_t)(PID_Caculate(&chassis->climb_pid,send_y,0) * 1000.f);
    float world_angle = DEG2RAD(chassis->ChassisPosReal.angle - chassis->climb_angle);
    int16_t vx = roundf(output * sinf(world_angle) * 1000.f);
    int16_t vy = roundf(output * cosf(world_angle) * 1000.f);

    memcpy(Data, &tmp_w, sizeof(int16_t));
    memcpy(Data + 2, &vx, sizeof(int16_t));
    memcpy(Data + 4, &vy, sizeof(int16_t));

    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}
void chassis_Upstand(CHASSIS *chassis, uint8_t up_height)
{
    chassis->StandUp = 1;
    chassis->Status = CHASSIS_CLIMBOVER;
    HeaderPrepare(MASTER_CHASSIS_PUSHUP, 2, &Rxheader);
    Data[0] = 'C';
    switch (up_height)
    {
    case CHASSIS_HEIGHT_ZERO:
        Data[1] = 'Z';
        break;
    case CHASSIS_HEIGHT_200mm:
        Data[1] = 'L';
        break;
    case CHASSIS_HEIGHT_400mm:
        Data[1] = 'H';
        break;
    default:
        break;
    }
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}

void chassis_Up2R1(CHASSIS *chassis)
{
//这句不对 删了
    chassis->Status = CHASSIS_CLIMBOVER;
    HeaderPrepare(MASTER_CHASSIS_BETHEONE, 2, &Rxheader);
    Data[0] = 'R';
    Data[1] = 'D';
    chassis->upstandOnR1 = true;
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}
void chassis_StepbyStep(CHASSIS *chassis, bool step_flag)
{
    chassis->Status = step_flag ? CHASSIS_CLIMBOVER : CHASSIS_RUN;
    HeaderPrepare(MASTER_CHASSIS_STEPCONTROL, 2, &Rxheader);
    Data[0] = 'S';
    Data[1] = step_flag;

    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}
// void chassis_Soleniod(bool solenoid_flag)
// {
//     HeaderPrepare(MASTER_CHASSIS_DOWNR1, 2, &Rxheader);
//     Data[0] = 'S';
//     Data[1] = solenoid_flag;

//     if (!CAN_Queue_IfFull(&CAN2_Txqueue))
//         CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
//     else
//         Can2FullFlag++;
//     return;
// }

void chassis_DownfromR1(CHASSIS *chassis)
{
    HeaderPrepare(MASTER_CHASSIS_DOWNR1, 2 ,&Rxheader);
    Data[0] = 'D';
    Data[1] = 'R';
    chassis->upstandOnR1 = false;
    if (!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue, Rxheader, Data);
    else
        Can2FullFlag++;
    return;
}
void chassis_Movehorizontal(CHASSIS *chassis,bool pos_set,u8 dis)
{
    HeaderPrepare(MASTER_CHASSIS_MOVEH,2,&Rxheader);
    Data[0] = pos_set;
    Data[1] = dis;
    if(!CAN_Queue_IfFull(&CAN2_Txqueue))
        CAN_Enqueue(&CAN2_Txqueue,Rxheader,Data);
    else
        Can2FullFlag++;
    return;
}
