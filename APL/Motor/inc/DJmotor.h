#pragma once

#include "main.h"
#include "pid.h"
#include "stdbool.h"

#define M3508_NUM 8
#define M2006_NUM 0
// 其和必须为4或8
#define Reduction_Ratio 19
#define USE_DJNUM 8 // 8或4
#define M2006_RATIO 36
#define M3508_RATIO 19.20320855f
// 两个电机减速比
#define Zero_Distance 15
enum DJmotor_mode
{

    DJ_RPM,      // 速度模式
    DJ_Position, // 位置模式
    DJ_Zero,     // 寻零模式
    DJ_Current   // 电流（力矩）模式

};

typedef struct
{

    volatile int16_t Current;    // 设定电流
    volatile float Angle;        // 角度制
    volatile int16_t speed;      // 速度
    volatile float Current_A;    // 换算后电流
    volatile int16_t PulseRead;  // 读入的脉冲数
    volatile int16_t PulseGap;   // 脉冲数差距
    volatile int32_t PulseTotal; // 脉冲数总和
    volatile int8_t temperature; // 电机温度 有些电机会反馈
} DJmotorVal;

// typedef struct
// {

// }DJmotorArgue;
typedef struct
{

    uint16_t PulsePerRound; // 电机每转一圈的脉冲数 常用8191
    float Gear_ratio;       // 机构减速比
    float Reduction_ratio;  // 电机减速比
    uint32_t ParamID;       // 电机本身ID号
    int16_t CurrentLimit;   // 电流限制

} DJmotorParam;

typedef struct
{
    // 标志位设置
    bool RPMLimitFlag;      // 速度模式限制标志
    bool PosAngleLimitFlag; // 位置模式角度限制标志
    bool PosRPMFlag;        // 位置模式速度限制标志位
    bool CurrentLimitFlag;  // 电流限制位
    // 数值设置
    float MaxAngle; // 最大角度
    float MinAngle; // 最小角度

    int16_t SpeedRPMLimit;
    int32_t PosRPMLimit;      // 位置模式转速限制
    int16_t ZeroRPMLimit;     // 寻零模式速度限制
    int16_t ZeroCurrentLimit; // 寻零模式电流限制
    bool IsLooseStuck;        // 是否堵转
} DJmotorLimit;

typedef struct
{

    volatile bool ZeroFlag;     // 寻零标志位
    volatile bool Overtimeflag; // 超时标志位
    volatile bool StuckFlag;    // 堵转标志位
    volatile bool IsSetZero;    // 位置置零标志位
} DJmotorStatus;

typedef struct
{
    volatile int32_t pulseLock; // 锁点位置累计脉冲
    uint16_t zeroCnt;           // 寻零计数器
    uint16_t GapCnt;
} DJmotorArgum;

typedef struct
{
    volatile uint32_t lastRxTime; // can接收超时计数
    uint16_t stuckCount;          // 堵转计数
    uint16_t timeoutCount;        // 超时计数
} DJmotorError;
typedef struct
{
    uint8_t ID;           // 电机顺序ID
    volatile bool Enable; // 使能
    volatile bool Begin;  // 开始
    uint8_t MODE;         // 模式：速度、位置、寻零

    DJmotorParam param;
    DJmotorVal valSet, valNow, valPre;
    DJmotorStatus statusflag;
    DJmotorLimit limit;
    DJmotorArgum argum;
    DJmotorError error;
    PIDType posPID, rpmPID; // 位置PID和转速PID
} *DJMotorPointer, DJMotor;

extern DJMotor DJmotor[USE_DJNUM];

void DJAngleCalculate(DJMotorPointer motor);                               // 角度值的计算
void DJMotorInit(void);                                                    // 电机的初始化
void DJSetZero(DJMotorPointer motor);                                      // 电机参数置零
void DJReceiveData_CAN1(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data); // 电机接收数据
void DJSpeedMode(DJMotorPointer motor);                                    // 速度模式
void DJPositionMode(DJMotorPointer motor);                                 // 位置模式
void DJCurrentTransmit(DJMotorPointer motor);                              // 电流量发送
void DJFunc(void);                                                         // 对外接口调用函数
void DJZeroMode(DJMotorPointer motor);                                     // 寻零模式
void DJLockPosition(DJMotorPointer motor);                                 // 电机锁位
