#ifndef __CHASSISPARA_H__
#define __CHASSISPARA_H__

#include "includes.h"
#include "Zdrive.h"
#include "Vescmotor.h"
#include "DJmotor.h"
#define FL 0
#define FR 3
#define BL 1
#define BR 2
//255.97mm
#define ZMDR_CHASSIS 1
#define VESC_CHASSIS 0
#define DJ_CHASSIS   0
#define SteeringWheel 1
#define WHEEL2CENTER 0.325269119345f //TODO 中心与轮距
#define WHEEL_R 0.04325f   //TODO　轮半径
#define carVel2RPM (60 / (2 * PI * WHEEL_R))

#define SIN_ANGLE_FL 0.70710678f //135
#define SIN_ANGLE_FR 0.70710678f  // 45
#define SIN_ANGLE_BR -0.70710678f // -135
#define SIN_ANGLE_BL -0.70710678f  // 45

#define COS_ANGLE_FL -0.70710678f // 135
#define COS_ANGLE_FR 0.70710678f // 45
#define COS_ANGLE_BR 0.70710678f  // -135
#define COS_ANGLE_BL -0.70710678f  // -45
//Vrpm = Vm/s * 60 / (2 * PI * WHEELR)

typedef enum
{
  RUN_NORMAL,
  RUN_CROSSBRAKE,
  RUN_PREFORNEXT
}CHASSIS_RUNMODE;

typedef struct _MOTORDATA
{
  volatile float *angle;   // 电机角度
  volatile float *current; // 电机电流
	#if DJ_CHASSIS
		volatile int16_t *speed;
	#else
		volatile float *speed;   // 电机速度
	#endif

} MOTORDATA;
typedef struct _WHEEL
{ 
    MOTORDATA DriveMotorValueReal;//驱动电机
    MOTORDATA SteerMotorValueReal;//转向电机
    #if ZMDR_CHASSIS
        Zdrive *wheelMotor;
    #endif
    #if VESC_CHASSIS
        VescMOTOR *wheelMotor;
    #endif
    #if DJ_CHASSIS
        DJMotor *wheelMotor;
    #endif
    #if SteeringWheel
        Zdrive *steerMotor;
    #endif 
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
    float crossAngle;

}WHEEL;
typedef struct _POSTURE
{
    float vx;
    float vy;
    float v;
    float w;

    int16_t x;
    int16_t y;
    int16_t s;
    float angle;
}POSTURE;

typedef struct _CHASSIS
{
    bool Enable;
    // volatile bool crossBrake; //是否在插锁
    // volatile bool IsRunningTraj; // 是否在跑线
    // volatile bool TurnForNextRunResearch; // 准备好进入下一阶段
    // volatile bool LockPoint;//是否在锁点
    volatile bool Ascend;//是否在上升
    volatile bool Descend; //是否在下降
    volatile bool StandUp; // 是否在起立
    volatile bool ClimbUp2R1;
    volatile bool ClimbBackDown;
		WHEEL wheel[4];

    POSTURE ChassisPosReal;
    POSTURE ChassisPosSet;

    float cosDirection;
    float sinDirection;
    
} CHASSIS;
typedef struct _THRESHOLD
{
	float UseTrajThreshold;
	float TrajDoneThreshold;
	float ChassisMaxSpeed;
//TODO 轨迹姿态相关 
}THRESHOLD;
extern CHASSIS Chassis;
extern CHASSIS_RUNMODE ChassisRun;
extern THRESHOLD Threshold;


#endif /* __CHASSISPARA_H__ */
