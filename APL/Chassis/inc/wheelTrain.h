#ifndef __WHEELTRAIN_H__
#define __WHEELTRAIN_H__

#include "includes.h"
#include "chassisPara.h"
#include "chassisComm.h"
#include "vector.h"

// 转向电机急转速度阈值
#define STEER_SHARP_TURN_SPEED_THRESHOLD 2.f
// 直接加速阈值，若速度差小于该阈值，则直接设定速度为匀速段速度
#define DIRECT_SPEEDUP_THRESHOLD 0.1f
// 手动模式车速限制
#define CHASSIS_MANUAL_MAX_VELOCITY 1.75f
// 手动模式角速度限制
#define CHASSIS_MANUAL_MAX_ANGULAR_VELOCITY 300.f
// 自动模式车速限制
#define CHASSIS_SLOPE_MAX_VELOCITY 2.f
// 冲坡车速限制
#define CHASSIS_MAX_VELOCITY 3.f
// 自动模式车角速度限制 单位°/s
#define CHASSIS_MAX_ANGULAR_VELOCITY 300
// 车最大加速度
#define CHASSIS_MAX_ACCELERATION 5.f
// 锁点角度误差允许值
#define CHASSIS_LOCK_ANGLE_THRESHOLD 1.0f
// 锁点位置误差允许值
#define CHASSIS_LOCK_POS_THRESHOLD_X 2.f
#define CHASSIS_LOCK_POS_THRESHOLD_Y 2.f
// 锁点最大速度
#define CHASSIS_LOCK_MAX_SPEED 2.2f
// 锁点最大角速度
#define CHASSIS_LOCK_MAX_ANGULAR_SPEED 200.f

#define CHASSIS_CLIMB_YAWCORRECT_SPEED 40.f
void Chassis_Init(CHASSIS *chassis);
int wheelTurnMin(WHEEL *wheel, float targetAngle);
void Chassis_carvelSet(CHASSIS *chassis);
void CalSingWheelSpeed(WHEEL *wheel, float carVxset, float carVyset, float carVw);
// void sendCtrlMsg(CHASSIS *chassis);
//  void crossLock(CHASSIS *chassis);
#endif /* __WHEELTRAIN_H__ */
