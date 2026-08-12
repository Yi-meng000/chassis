#include "ZDrive.h"
#include "stdbool.h"
#include "stdlib.h"
#include "FD_Canqueue.h"
#include "mathFunc.h"

Zdrive Zmotor[USE_ZDRIVE_NUM];
void ZdriveInit()
{
    for (int i = 0; i < USE_ZDRIVE_NUM; i++)
    {
        Zmotor[i].param.GearRatio = 1.f;
        Zmotor[i].param.ReductionRatio = 18.36f; // J60减速比
        Zmotor[i].valSetNow.speed = 0;
        Zmotor[i].valSetNow.angle = 0;
        Zmotor[i].statusflag.Arriveflag = false;
        Zmotor[i].argum.GapCnt = 0;
        Zmotor[i].valReal.angle = 0;
        Zmotor[i].param.kd = 10.f;
        Zmotor[i].param.kp = 2.3f;
        Zmotor[i].mode = Zdrive_Postion;
        Zmotor[i].Enable = 0;
        Zmotor[i].Begin = 0;
        Zmotor[i].statusflag.ZeroPoint = 0;
        Zmotor[i].statusflag.Zeroflag = 0;
    }
}
void ZdriveSetPI(float p_p, float v_p, uint8_t id)
{
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x21;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &p_p, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x21;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &v_p, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
            return;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x25;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &p_p, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
            return;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x25;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &v_p, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveSetPosition(float angle, uint8_t id)
{

    angle /= 360.f;  // 角度转换
    angle *= 18.36f; // 减速比
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x47;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &angle, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
        {
            return;
        }
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x47;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &angle, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveSetSpeed(float speed, uint8_t id)
{

    speed /= 60.f;
    speed *= 18.36f;
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x45;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &speed, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
            return;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x45;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &speed, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveSetMode(float mode, uint8_t id)
{
    Zmotor[id - 1].mode = mode; //
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x3D;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &mode, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
            return;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x3D;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &mode, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveSetTorque(uint8_t id)
{
    float torque_base = 0.13; // TODO 赋值
    float torque = Zmotor[id - 1].param.kp * ((Zmotor[id - 1].valSetNow.angle - Zmotor[id - 1].valReal.angle) / 180 * PI) - (Zmotor[id - 1].param.kd * (Zmotor[id - 1].valSetNow.speed - Zmotor[id - 1].valReal.speed) / 180 * PI);
    if (torque > 0)
        torque += torque_base;
    else
        torque -= torque_base;
    // T = T0 + kp * (Angle - Angle_Now) + kd * (Velocity - Velocity_Now);
    float current = 0.7422f * (torque);
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x43;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &current, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
            return;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x43;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &current, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveSetPVT(float speed, float angle, uint8_t id)
{

    angle *= (18.36f / 360.f);
    speed *= (18.36f);
    double temp;
    temp = cvtFloat2Double(speed, angle);
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
        {
            Can1FullFlag++;
            return;
        }

        memcpy(CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data, &temp, sizeof(double));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 8;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
        {
            Can2FullFlag++;
            return;
        }
        memcpy(CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data, &temp, sizeof(double));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 8;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveSetPresentPos(float angle, uint8_t id)
{
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
    {
        return;
    }
    angle /= 360.f; // 设置角度以圈为单位
    angle *= 18.36f;
    if (id > 0 && id <= 4)
    {
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x5f;
        memcpy(&CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1], &angle, sizeof(float));
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 5;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x5f;
        memcpy(&CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1], &angle, sizeof(float));
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 5;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveClearErr(uint8_t id)
{

    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
            return;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = 0x41;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[1] = 0x0;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 1;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
            return;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = 0x41;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[1] = 0x0;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 1;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}
void ZdriveReceive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data)
{
    uint8_t controlID = Rxheader.Identifier;
    uint8_t operationID = Rx_Data[0];

    if (controlID > 12 && controlID <= 20)
    {
        controlID -= 12;
        Zmotor[controlID - 1].statusflag.err = Zdrive_Well;
        float pos_tmp;
        int16_t vel_tmp;
        memcpy(&pos_tmp, &Rx_Data[0], sizeof(float));
        Zmotor[controlID - 1].valReal.angle = N2DEG(pos_tmp / Zmotor->param.ReductionRatio);
        memcpy(&vel_tmp, &Rx_Data[4], sizeof(int16_t));
        Zmotor[controlID - 1].valReal.speed = N2DEG(((float)(vel_tmp) * 0.002f)) / 6.f / Zmotor->param.ReductionRatio;
        return;
    }
    else if (controlID <= 8)
    {
        Zmotor[controlID - 1].statusflag.err = Zdrive_Well;
        switch (operationID)
        {
        case 0x5E: // position
        {
            memcpy(&Zmotor[controlID - 1].valReal.angle, &Rx_Data[1], sizeof(float));
            Zmotor[controlID - 1].valPre.angle = Zmotor[controlID - 1].valReal.angle;
            Zmotor[controlID - 1].valReal.angle *= (360.f / Zmotor[controlID - 1].param.ReductionRatio);
            break;
        }
        case 0x52: // current
        {
            memcpy(&Zmotor[controlID - 1].valReal.current, &Rx_Data[1], sizeof(float));
            Zmotor[controlID - 1].valReal.Torque = 1.3473f * Zmotor[controlID - 1].valReal.current;
            break;
        }
        case 0x5C: // speed
        {
            memcpy(&Zmotor[controlID - 1].valReal.speed, &Rx_Data[1], sizeof(float));
            Zmotor[controlID - 1].valReal.speed /= Zmotor[controlID - 1].param.ReductionRatio;
            break;
        }
        case 0x3C: // mode
        {
            float tempMode;
            memcpy(&tempMode, &Rx_Data[1], sizeof(float));
            Zmotor[controlID - 1].modeRead = (ZdriveMode)tempMode;
            break;
        }
        case 0x40: // error
        {
            float tempErr;
            memcpy(&tempErr, &Rx_Data[1], sizeof(float));
            Zmotor[controlID - 1].statusflag.err = (ZdriveErr)tempErr;
            break;
        }
        case 0x46: // pos_in
        {
            float tempPos_in;
            memcpy(&tempPos_in, &Rx_Data[1], sizeof(float));
            Zmotor[controlID - 1].valReal.posIn = tempPos_in * 360.f / Zmotor[controlID - 1].param.ReductionRatio;
            break;
        }
        default:
            break;
        }
    }
}

void ZdriveAsk(uint8_t id, uint8_t ask_code)
{
    if (id > 0 && id <= 4)
    {
        if (CAN_Queue_IfFull(&CAN1_Txqueue))
        {
            return;
        }
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].Data[0] = ask_code;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].ID = id;
        CAN1_Txqueue.FDCAN_DataSend[CAN1_Txqueue.Rear].DLC = 1;
        CAN1_Txqueue.Rear = (CAN1_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
    else
    {
        if (CAN_Queue_IfFull(&CAN2_Txqueue))
        {
            return;
        }
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].Data[0] = ask_code;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].ID = id;
        CAN2_Txqueue.FDCAN_DataSend[CAN2_Txqueue.Rear].DLC = 1;
        CAN2_Txqueue.Rear = (CAN2_Txqueue.Rear + 1) % FDCAN_QUEUESIZE;
    }
}

void ZdrivePVT_Cal(uint8_t id)
{
    float err_angle = (Zmotor[id - 1].valSetNow.angle - Zmotor[id - 1].valSetPre.angle) / 360.f;
    Zmotor[id - 1].valSetNow.speed = err_angle / (float)Zdrive_Hz * 1000.f;
    if (ABS(Zmotor[id - 1].valSetNow.speed) > 0.5f)
        Zmotor[id - 1].valSetNow.speed = GetSign(Zmotor[id - 1].valSetNow.speed) * 0.5f;
    return;
}
void ZdriveFunc()
{
    for (int i = 0; i < USE_ZDRIVE_NUM; i++)
    {
        if (Zmotor[i].Enable)
        {
            if (Zmotor[i].Begin)
            {
                switch (Zmotor[i].mode)
                {
                case Zdrive_Speed:
                {
                    ZdriveSetSpeed(Zmotor[i].valSetNow.speed, i + 1);
                    break;
                }
                case Zdrive_Current:
                {
                    ZdriveSetTorque(i + 1);
                    break;
                }
                case Zdrive_Postion:
                {
                    if (Zmotor[i].valSetNow.angle != 0)
                    {
                        float z = Zmotor[i].valSetNow.angle - Zmotor[i].valSetPre.angle;
                        if (ABS(z) > 1e-4f)
                        {
                            // 当角度的偏差过大
                            // ZdriveSetPosition(Zmotor[i].valSetNow.angle,i+1);
                            ZdrivePVT_Cal(i + 1);
                            ZdriveSetPVT(Zmotor[i].valSetNow.speed, Zmotor[i].valSetNow.angle, i + 1);
                        }
                    }
                    else if (ABS(Zmotor[i].valPre.angle) > 0.1f)
                    {
                        ZdrivePVT_Cal(i + 1);
                        ZdriveSetPVT(Zmotor[i].valSetNow.speed, Zmotor[i].valSetNow.angle, i + 1);
                    }
                    Zmotor[i].valSetPre.angle = Zmotor[i].valSetNow.angle;
                    break;
                }
                default:
                    break;
                }
            }
            if (Zmotor[i].modeRead != Zmotor[i].mode)
            {
                ZdriveSetMode((float)Zmotor[i].mode, i + 1);
                Zmotor[i].modeRead = Zmotor[i].mode;
            }
            if (ABS(Zmotor[i].valSetNow.angle) > 270.f)
                Zmotor[i].Enable = 0;
        }
        if (Zmotor[i].statusflag.Zeroflag)
        {
            // ZdriveSetPresentPos(0,i+1);
            // ZdriveSetPresentPos(0,i+1);
            ZdriveSetPresentPos(Zmotor[i].statusflag.ZeroPoint, i + 1);
            Zmotor[i].statusflag.Zeroflag = 0;
        }
        if (Zmotor[i].valReal.current > 18.f)
            Zmotor[i].mode = 0;
    }
    //
}
