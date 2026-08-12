#include "DJmotor.h"
#include "main.h"
#include "mathFunc.h"
#include "fdcan.h"

DJMotor DJmotor[USE_DJNUM];
/**
 * @brief 初始化
 *
 */
void DJMotorInit()
{
    DJmotorParam DJ2006Param, DJ3508Param;
    // 两个不同型号的电机参数
    DJmotorLimit limit;
    // 输出限制
    DJmotorStatus statusflag;
    // 标志位
    DJmotorArgum argum;
    DJmotorError error;
    DJ2006Param.ParamID = 0x1ff;
    DJ2006Param.Gear_ratio = 1;
    DJ2006Param.Reduction_ratio = M2006_RATIO;
    DJ2006Param.PulsePerRound = 8191;
    DJ2006Param.CurrentLimit = 4500;

    DJ3508Param.ParamID = 0x200;
    DJ3508Param.Gear_ratio = 1;
    DJ3508Param.Reduction_ratio = M3508_RATIO;
    DJ3508Param.PulsePerRound = 8191;
    DJ3508Param.CurrentLimit = 10000;

    limit.CurrentLimitFlag = true;
    // 默认开启限流
    limit.IsLooseStuck = false;
    // 不堵转

    // 位置模式
    limit.MaxAngle = 270;
    limit.MinAngle = -270;
    limit.PosAngleLimitFlag = false;
    limit.PosRPMLimit = true;
    limit.PosRPMLimit = 2000;

    // 速度模式
    limit.RPMLimitFlag = false;
    limit.SpeedRPMLimit = 10000;
    // 寻零模式
    limit.ZeroCurrentLimit = 3000;
    limit.ZeroRPMLimit = 500;

    // 状态设置
    statusflag.IsSetZero = true;
    statusflag.Overtimeflag = false;
    statusflag.StuckFlag = false;
    statusflag.ZeroFlag = false;

    argum.pulseLock = 0;
    argum.zeroCnt = 0;
    argum.GapCnt = 0;

    error.lastRxTime = 0;
    error.stuckCount = 0;
    error.timeoutCount = 0;

    for (int i = 0; i < USE_DJNUM; i++)
    {
        DJmotor[i].Begin = false;
        DJmotor[i].Enable = false;
        DJmotor[i].statusflag = statusflag;
        DJmotor[i].limit = limit;
        DJmotor[i].argum = argum;
        DJmotor[i].error = error;
        DJmotor[i].valNow.PulseTotal = 0;
        DJmotor[i].valPre.PulseRead = 0;
    }
    // for(int i =0; i < M2006_NUM ; i++)
    // {
    //     DJmotor[i].ID = i+1;
    //     DJmotor[i].MODE = DJ_Position;//todo
    //     DJmotor[i].param = DJ2006Param;
    // }
    for (int i = 0; i < M3508_NUM; i++)
    {
        DJmotor[i].ID = i + 1;         // 从1开始
        DJmotor[i].MODE = DJ_Position; // 默认是位置模式
        DJmotor[i].param = DJ3508Param;
    }
    for (int i = 0; i < USE_DJNUM; i++)
    {
        PID_Init(&DJmotor[i].posPID, 2, 0.08, 0, 0);
        PID_Init(&DJmotor[i].rpmPID, 8, 0.3, 0, 0);
    }
    // PID参数初始化
}
void DJSetZero(DJMotorPointer motor)
{
    motor->statusflag.IsSetZero = false; // 标志位
    motor->valNow.Angle = 0;             // 角度归零
    motor->valNow.PulseTotal = 0;        // 脉冲总数归零
    motor->argum.pulseLock = 0;
}
void DJReceiveData_CAN1(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    // 收到的ID只会在201至208内
    // 而发出的ID要考虑的就多了
    if (Rxheader.Identifier >= 0x201 && Rxheader.Identifier <= 0x208)
    {
        uint8_t Card_ID = Rxheader.Identifier - 0x200; // 得到其顺序
        for (int i = 0; i < USE_DJNUM; i++)
        {
            if (Card_ID == DJmotor[i].ID)
            {
                DJmotor[i].valNow.PulseRead = (Rx_data[0] << 8) | Rx_data[1];
                DJmotor[i].valNow.speed = (Rx_data[2] << 8) | Rx_data[3];
                DJmotor[i].valNow.Current = (Rx_data[4] << 8) | Rx_data[5];
                // 左移8位进入高位 或上 低8位

                if (DJmotor[i].param.Reduction_ratio == M3508_RATIO)
                {
                    DJmotor[i].valNow.temperature = Rx_data[6]; // M3508独有的返回温度
                    DJmotor[i].valNow.Current_A = (float)DJmotor->valNow.Current * 0.0012207f;
                    // 电流范围-16384 至 16384 以A为单位则-20A 至20A 即除以16384再乘20
                }
                else
                {
                    DJmotor[i].valNow.Current_A = (float)DJmotor->valNow.Current / 10000 * 10;
                    // 电流范围-10000至10000 -10A~10A
                }
            }
            DJAngleCalculate(&DJmotor[i]);
            // 将受到数据转化为电机现实角度
        }
    }
}
void DJAngleCalculate(DJMotorPointer motor)
{
    motor->valNow.PulseGap = motor->valNow.PulseRead - motor->valPre.PulseRead;
    // 计算脉冲的差值
    if (ABS(motor->valNow.PulseGap) > 4096) // 判断电机该正转还是反转
        motor->valNow.PulseGap -= GetSign(motor->valNow.PulseGap) * motor->param.PulsePerRound;
    // 若大于半圈（一圈为8192） 则减为小的半圈
    motor->valNow.PulseTotal += motor->valNow.PulseGap;
    // 总脉冲数加上脉冲的差值
    motor->valNow.Angle = motor->valNow.PulseTotal * 360 / (motor->param.PulsePerRound * motor->param.Gear_ratio * motor->param.Reduction_ratio);
    // 角度 = 总脉冲数 /(每圈脉冲数*减速比*机构减速比/360)
    if (motor->Begin)
        motor->argum.pulseLock = motor->valNow.PulseTotal; // 注意把脉冲锁更新
    if (motor->statusflag.IsSetZero)
    {
        DJSetZero(motor); // 未归零则归零
        motor->statusflag.IsSetZero = 0;
    }
    motor->valPre = motor->valNow; // 更新valpre
}
void DJLockPosition(DJMotorPointer motor)
{
    motor->posPID.SetVal = motor->argum.pulseLock;
    motor->posPID.CurVal = motor->valNow.PulseTotal;

    motor->rpmPID.SetVal = PID_Caculate(&motor->posPID, motor->posPID.CurVal, motor->posPID.SetVal);
    // 直接赋值 速度为位置导数
    motor->rpmPID.CurVal = motor->valNow.speed;

    motor->valSet.Current += PID_Caculate(&motor->rpmPID, motor->rpmPID.CurVal, motor->rpmPID.SetVal);
    // 累加
    PeakLimit(motor->valSet.Current, 3000);
}
void DJSpeedMode(DJMotorPointer motor)
{
    motor->rpmPID.SetVal = motor->valSet.speed;
    motor->rpmPID.CurVal = motor->valNow.speed;
    if (motor->limit.RPMLimitFlag)
        PeakLimit(motor->rpmPID.SetVal, motor->limit.SpeedRPMLimit);
    // 多加个保险
    motor->valSet.Current += PID_Caculate(&motor->rpmPID, motor->rpmPID.CurVal, motor->rpmPID.SetVal);
}

void DJPositionMode(DJMotorPointer motor)
{
    motor->valSet.PulseTotal = motor->valSet.Angle * motor->param.Gear_ratio * motor->param.Reduction_ratio * motor->param.PulsePerRound / 360;
    // 总脉冲数 = 设定角度 * 两个减速比 * 每圈的脉冲数 /360
    if (motor->limit.PosAngleLimitFlag) // 有角度限制
    {
        int32_t MaxPulse = motor->limit.MaxAngle * motor->param.PulsePerRound * motor->param.Gear_ratio * motor->param.Reduction_ratio / 360;
        int32_t MinPulse = motor->limit.MinAngle * motor->param.PulsePerRound * motor->param.Gear_ratio * motor->param.Reduction_ratio / 360;
        // 计算最大最小角度
        if (motor->valSet.PulseTotal > MaxPulse)
            motor->posPID.SetVal = MaxPulse;
        if (motor->valSet.PulseTotal < MinPulse)
            motor->posPID.SetVal = MinPulse;
        else
            motor->posPID.SetVal = motor->valSet.PulseTotal;
        // posPID.SetVal 以脉冲数为单位
    }
    else
        motor->posPID.SetVal = motor->valSet.PulseTotal;
    motor->posPID.CurVal = motor->valNow.PulseTotal;

    motor->rpmPID.SetVal = PID_Caculate(&motor->posPID, motor->posPID.CurVal, motor->posPID.SetVal);
    // PID计算设定速度
    motor->rpmPID.CurVal = motor->valNow.speed;
    if (motor->limit.PosRPMFlag)
    {
        PeakLimit(motor->rpmPID.SetVal, motor->limit.PosRPMLimit);
    }
    motor->valSet.Current += PID_Caculate(&motor->rpmPID, motor->rpmPID.CurVal, motor->rpmPID.SetVal);
}
void DJZeroMode(DJMotorPointer motor)
{
    motor->rpmPID.SetVal = motor->limit.ZeroRPMLimit;
    // 以稳定低速转
    motor->rpmPID.CurVal = motor->valNow.speed;
    motor->valSet.Current += PID_Caculate(&motor->rpmPID, motor->rpmPID.CurVal, motor->rpmPID.SetVal);
    PeakLimit(motor->valSet.Current, motor->limit.ZeroCurrentLimit);

    // 寻零判断
    if (ABS(motor->valNow.PulseGap) < Zero_Distance)
    {

        if (motor->argum.zeroCnt++ > 100)
        {
            motor->argum.zeroCnt = 0;

            motor->statusflag.ZeroFlag = true;
            motor->Begin = false;
            DJSetZero(motor);
        }
    }
}
void DJCurrentTransmit(DJMotorPointer motor)
{
    uint8_t tag = 0;
    static uint8_t Tx_data[8] = {0};
    static FDCAN_TxHeaderTypeDef Txheader = {0};
    PeakLimit(motor->valSet.Current, motor->param.CurrentLimit);
    // 限流
    if (motor->Enable != true)
        motor->valSet.Current = 0;
    // 未使能置零电流
    Txheader.IdType = FDCAN_STANDARD_ID;
    Txheader.TxFrameType = FDCAN_DATA_FRAME;
    Txheader.DataLength = 8;
    if (motor->ID <= 4)
    {
        Txheader.Identifier = 0x200;
        tag = (motor->ID - 1) * 2;
    }
    else
    {
        Txheader.Identifier = 0x1FF;
        tag = (motor->ID - 5) * 2; // ID的换算
    }
    EncodeS16Data(&motor->valSet.Current, &Tx_data[tag]); // float转带符号16位数
    ChangeDataByte(&Tx_data[tag], &Tx_data[tag + 1]);     // 高低位互换

    if (motor->ID == 4 || motor->ID == 8) // 积攒到4个或8个再发出，提高传递信息效率
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txheader, Tx_data);
}
void DJMonitor(DJMotorPointer motor)
{
    if (motor->Begin && motor->Enable)
    {

        if (motor->valNow.PulseGap < 5 && motor->valNow.Current > 3000)
        {
            if (motor->error.stuckCount++ > 500)
            {
                motor->error.stuckCount = 0;
                if (motor->limit.IsLooseStuck)
                    motor->Enable = false;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            }
        }
        else
            motor->error.stuckCount = 0;

        if (motor->error.lastRxTime++ > 20)
            if (motor->error.timeoutCount++ > 10)
            {
                motor->error.timeoutCount = 0;
                motor->Enable = false;
                // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
            }
    }
}
void DJFunc()
{
    for (int i = 0; i < USE_DJNUM; i++)
    {
        if (DJmotor[i].Enable)
        {
            if (DJmotor[i].Begin)
            {
                switch (DJmotor[i].MODE)
                {
                case DJ_RPM:
                    DJSpeedMode(&DJmotor[i]);
                    break;
                case DJ_Position:
                    DJPositionMode(&DJmotor[i]);
                    break;
                case DJ_Zero:
                    DJZeroMode(&DJmotor[i]); // 注意是DJmotor[i]
                    break;
                case DJ_Current:
                    break;
                default:
                    break;
                }
                if (DJmotor[i].argum.GapCnt++ > 1000)
                {
                    DJmotor[i].argum.GapCnt = 0;
                    DJSetZero(&DJmotor[i]);
                    DJmotor[i].valSet.Angle = 360;
                }
            }
            else
                DJLockPosition(&DJmotor[i]); // 注意其摆放位置
        }
        DJCurrentTransmit(&DJmotor[i]);
    }
}
