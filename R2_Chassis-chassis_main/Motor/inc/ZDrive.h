#ifndef __ZDRIVE_H__
#define __ZDRIVE_H__

#include "main.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "FD_Canqueue.h"
#define USE_ZDRIVE_NUM 14
#define Zdrive_Hz 10

#define POU 10000.f
#define POD -10000.f
#define Velocity_Limit 150.f
#define Current_Limit 40.f 

#define PID_POS_P   0x12
#define PID_POS_I   0x13
#define PID_VEL_P   0x14
#define PID_VEL_I   0x15

typedef enum
{
    Zdrive_Disable = 0,                  // 0失能
    Zdrive_Current,                  // 1电流模式
    Zdrive_Speed,                    // 2速度模式
    Zdrive_Postion,                  // 3位置模式
    Zdrive_Test,                     // 4测试模式
    Zdrive_RVCalibration,            // 5电阻电感校准
    Zdrive_EncoderLineCalibration,   // 6编码器线性补偿
    Zdrive_EncoudeOffsetCalibration, // 7编码器偏移校准
    Zdrive_VKCalibration,            // 8VK校准
    Zdrive_SaveSetting,              // 9保存配置
    Zdrive_EraseSetting,             // 10擦除配置
    Zdrive_ClearErr,                 // 11擦除错误
    Zdrive_Brake                     // 12刹车
} ZdriveMode;

typedef enum
{
    Zdrive_Well = 0,                // 0无
    Zdrive_InsufficientVoltage, // 1低电压
    Zdrive_OverVoltage,         // 2过电压
    Zdrive_InstabilityCurrent,  // 3电流不稳
    Zdrive_OverCurrent,         // 4过电流
    Zdrive_OverSpeed,           // 5超速
    Zdrive_ExcessiveR,          // 6电阻过大
    Zdrive_ExcessiveInductence, // 7电感过大
    Zdrive_LoseEncoder1,        // 8编码器错误
    Zdrive_PolesErr,            // 9极对数不匹配
    Zdrive_VKCalibrationErr,    // 10 KV校准失败
    Zdrive_ModeErr,             // 11模式不合法
    Zdrive_ParameterErr,        // 12参数错误
    Zdrive_Hot                  // 13过热
} ZdriveErr;

typedef enum
{
    SES = 0x01,//有无感
    ENCODER,//编码器类型
    ENL,//线性补偿
    NodeID,//节点iD
    CAN_HZ, // can频率
    HEARTBEAT, //心跳间隔
    Volta_LL, //电压下限
    Pos_UL, //位置上限
    Pos_LL, //位置下限
    Vel_Limit, // 速度限制
    Poles_Num,//极对数
    CurLimit, // 电流限制
    CurrCAL, //校准电流
    Start_Mode, // 启动模式
    Answer_Mode, //响应模式
    FilterCoeff, // 滤波系数
    ToleranceCoeff, //容忍系数
    Pos_PID_P, //
    Pos_PID_I, //
    Vel_PID_P, //速度
    Vel_PID_I, // 速度
    Acc_Acu, // 加速度
    Acc_Dec, // 减速度
    Pos_Vel_TimeGap, //速度位置模式时间间隔


    Mode = 0x1F, // 模式
    Warning, // 警告
    Err, // 错误
    CurIn, // 电流输入
    VelIn, // 速度输入
    PosIn, // 位置输入
    LIF, //队列控制 

    Cur_M = 0x2B,
    Vel = 0x2D, //当前转速
    Pur = 0x2E, //当前位置
    PVT_Frame = 0x39, //PVT模式

}ZdriveCmd;
 
typedef struct
{
    float speed;
    float angle;
    float posIn;
    float current;
    float Torque;
    float velLimit;  // current velocity limit from drive
    float accAcu;    // current acceleration limit from drive
    float accDec;    // current deceleration limit from drive
}ZdriveValue;

typedef struct
{
    uint16_t GapCnt;
    uint32_t lastRxtime;
    uint32_t timeoutTicks;
    uint32_t stuckCnt;
    float lockAngle;
}ZdriveArgum;
typedef struct  
{   
    float GearRatio;
    float ReductionRatio;
    float kp;
    float kd;
    /* data */
}ZdriveParam;

typedef struct
{
    bool timeoutCheck;
    bool stuckCheck;
    bool Arriveflag;
	bool Zeroflag;
	float ZeroPoint;
    ZdriveErr err;
}ZdriveStatus;
typedef struct 
{
   bool PVTflag;
   float Total_Time;
}ZdrivePVTParam;

typedef struct
{
    
    ZdriveMode mode,modeRead;
    ZdriveParam param;
    ZdriveArgum argum;
    ZdriveValue valReal,valPre,valSetNow,valSetPre;
    ZdriveStatus statusflag;
    ZdrivePVTParam pvtparam;
    bool Enable;
    bool Begin;
}Zdrive;
extern FDCAN_SendQueueType *ZdriveSendQueue;
extern Zdrive Zmotor[USE_ZDRIVE_NUM];

void ZdriveInit(void);
void ZdriveFunc(void);
void ZdriveReceive(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);
void ZdriveSet(float data,uint8_t id,uint8_t set_code);
void ZdriveAsk(uint8_t id,uint8_t ask_code);
void ZdriveSetPVT(float speed,float angle,uint8_t id); 
void ZdriveSetMIT(uint8_t id);
void ZdriveClearErr(uint8_t id);
void ZdriveSetVelLimit(float vel,uint8_t id);
void ZdriveSetPID(float value, uint8_t id, uint8_t pid_code);
void ZdriveSetPosVelLimit(float vel_limit, uint8_t id);
void ZdriveSetAccel(float ace, uint8_t id);

#endif /* __ZDRIVE_H_v_ */

