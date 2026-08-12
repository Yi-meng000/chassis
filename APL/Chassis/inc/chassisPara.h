#ifndef __CHASSISPARA_H__
#define __CHASSISPARA_H__

#include "includes.h"
#include "pid.h"
#define FL 0
#define FR 1
#define BR 2
#define BL 3
// 255.97mm

#define CHASSIS_SIDELEN 635
#define WHEEL2CENTER 0.33799f // TODO 中心与轮距
#define WHEEL_R 0.0865f       // TODO　轮半径
#define carVel2RPM (60 / (2 * PI * WHEEL_R))

#define SIN_ANGLE_FL 0.70710678f  // 135
#define SIN_ANGLE_FR 0.70710678f  // 45
#define SIN_ANGLE_BR -0.70710678f // -135
#define SIN_ANGLE_BL -0.70710678f // 45

#define COS_ANGLE_FL -0.70710678f // 135
#define COS_ANGLE_FR 0.70710678f  // 45
#define COS_ANGLE_BR 0.70710678f  // -135
#define COS_ANGLE_BL -0.70710678f // -45
// Vrpm = Vm/s * 60 / (2 * PI * WHEELR)

typedef enum
{
    CHASSIS_DISABLE,
    CHASSIS_RUN,
    CHASSIS_CLIMBOVER,
    CHASSIS_ERROR
} CHASSIS_STATUS;
typedef enum
{
    RUN_NORMAL,
    RUN_CROSSBRAKE,
    RUN_PREFORNEXT
} CHASSIS_RUNMODE;
typedef enum
{
    CHASSIS_HEIGHT_ZERO,
    // CHASSIS_HEIGHT_200mm,
    CHASSIS_HEIGHT_400mm,
    CHASSIS_HEIGHT_200mm,

} CHASSIS_HEIGHT;
typedef struct _MOTORDATA
{
    volatile float angle;   // 电机角度
    volatile float current; // 电机电流
    volatile float speed;   // 电机速度

} MOTORDATA;
typedef struct _WHEEL
{
    MOTORDATA DriveMotorValueReal; // 驱动电机
    MOTORDATA SteerMotorValueReal; // 转向电机

    float InitAngle;

    float VxSet; // x方向速度设置
    float VySet; // y方向速度设置
    float VSet;  // 合速度

    float xSet; // x方向位移设置
    float ySet; // y方向位移设置
    float rSet; // 合位移

    float angleSetRad; // 角度设置弧度制
    float angleSetDeg; // 角度设置角度制

    float cosPhaseAngle;
    float sinPhaseAngle;

} WHEEL;
typedef struct _POSTURE
{
    float vx;
    float vy;
    float v;
    float w;

    int16_t x;
    int16_t y;
    int16_t z;
    int16_t s;
    float angle;
} POSTURE;
typedef enum _SlopeClimbStatus
{
    SlopeBottom,
    SlopeBottomEdge,
    SlopeOn,
    SlopeTopEdge,
    SlopeTop,
} SlopeClimbStatus;

typedef struct _CHASSIS
{
    bool Enable;
    volatile bool crossBrake;             // 是否在插锁
    volatile bool IsRunningTraj;          // 是否在跑线
    volatile bool TurnForNextRunResearch; // 准备好进入下一阶段
    volatile bool LockPoint;              // 是否在锁点
    volatile bool CamLockPoint;           // 相机锁点
    volatile bool climbover;              // 是否在翻越
    volatile bool StandUp;                // 是否在提高
    volatile bool slopeover;              // 是否在爬坡
    volatile bool ClimbUp2R1;             // 是否在登上r1
    volatile float pitch;
    volatile bool upstandOnR1;
    volatile CHASSIS_STATUS Status;
    PIDType climb_pid;
    WHEEL wheel[4];

    POSTURE ChassisPosReal;
    POSTURE ChassisPosSet;

    float cosDirection;
    float sinDirection;
    bool sendCorrect_w;
    int16_t climb_record_x;
    int16_t climb_record_y;
    float climb_angle;
    SlopeClimbStatus slopeState;

} CHASSIS;

typedef struct _THRESHOLD
{
    float UseTrajThreshold;  //
    float TrajDoneThreshold; //
    float ChassisMaxSpeed;
    // TODO 轨迹姿态相关
} THRESHOLD;
typedef struct _lockangleVelThrehold
{
    float angularVelmax;
    float angularVelmin;
} LockangleVelThreholdTypedef;

typedef struct _lockpointThreshold
{
    float lockpointvelmax;
    float lockpointvelmin;
    float lockposthreshold;
    float lockanglethreshold;
    float lockpointTimeout;
} LockpointThresholdTypedef;

extern CHASSIS Chassis;

#endif /* __CHASSISPARA_H__ */
