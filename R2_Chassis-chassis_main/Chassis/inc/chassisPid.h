#ifndef __CHASSISPID_H__
#define __CHASSISPID_H__

#include "pid.h"
#include "chassisComm.h"
#include "chassisPara.h"
#include "wheelTrain.h"
#include "Trajectory.h"
#include "LED.h"
extern PIDType anglePID;
extern Vector2fPID trajPID;
extern Vector2fPID lockPointPID;
extern Bpoint testPoint;
extern float testangle;
extern float LockAnglePIDParam[3];
extern float TrajPIDParam[3];
extern float LockPointPIDParam[3];

void chassisLockAngle(CHASSIS *chassis,float aimAngle);
void chassisRunInit(CHASSIS  *chassis);
bool chassis_LockPoint(CHASSIS* chassis,Bpoint PosSet,float AngleSet,Vector2fPID *lockpointPid,
                        PIDType * lockanglePid);
void chassis_AimPoint(CHASSIS *chassis,Bpoint PosSet,PIDType *lockanglePid);
#endif /* __CHASSISPID_H__ */
