#ifndef __CHASSISPID_H__
#define __CHASSISPID_H__

#include "pid.h"
#include "chassisComm.h"
#include "chassisPara.h"
#include "wheelTrain.h"
#include "Trajectory.h"
#include "LED.h"

#define THRESHOLD_NUM 3
#define SLOPE_PITCHANGLE 15.f
#define CatchableBlock_Depth 820
typedef enum _PID_phase
{
    NormalPID,
    HarderPID,
    MediumHard,
} PID_Phase;
extern PIDType anglePID;
extern PIDType climbanglePID;
extern PIDType KeepanglePID;
extern Vector2fPID trajPID;
extern Vector2fPID lockPointPID[THRESHOLD_NUM];
extern Bpoint testPoint;
extern float testangle;
extern uint8_t testLockPID;
extern uint8_t testThrehold;
extern uint16_t CamDepth;
extern float LockAnglePIDParam[3];
extern float TrajPIDParam[3];
extern float LockPointPIDParam[3];

extern LockangleVelThreholdTypedef LockangleVelThreshold[THRESHOLD_NUM];
extern LockpointThresholdTypedef LockPointThreshold[THRESHOLD_NUM];

void chassisLockAngle(CHASSIS *chassis, float aimAngle);
void chassisRunInit(CHASSIS *chassis);
bool chassis_LockPoint(CHASSIS *chassis, Bpoint PosSet, float AngleSet, Vector2fPID *lockpointPid,
                       PIDType *lockanglePid, LockpointThresholdTypedef *lockpointThreshold);
void chassis_AimPoint(CHASSIS *chassis, Bpoint PosSet, PIDType *lockanglePid);
void chassis_ClimbPosture(CHASSIS *chassis, float aimAngle);
void chassis_LockPoint_ThresholdInit(CHASSIS *chassis, uint8_t thresholdNum, float velmax, float velmin, float posthrehold, float anglethrehold, uint16_t TimeoutCount);

bool Camera_LockPoint(CHASSIS *chassis, Bpoint PosReal, uint16_t Depth_X, float AngleSet, PIDType *lockanglePid, Vector2fPID *lockpointPid,
                      LockpointThresholdTypedef *lockpointThreshold);
#endif /* __CHASSISPID_H__ */
