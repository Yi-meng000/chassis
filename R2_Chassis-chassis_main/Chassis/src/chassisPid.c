#include "chassisPid.h"

float LockAnglePID[3] = {3.f,1.2f,0.f};
float TrajPIDParam[3] = {0.008f,0.0f,0.0f};
float LockPointPIDParam[3] = {0.0016f,0.001f,0.000f};
PIDType anglePID;
Vector2fPID trajPID;
Vector2fPID lockPointPID;

// void chassisRunInit(CHASSIS  *chassis)
// {
//     PID_Init(&anglePID,LockAnglePID[0],LockAnglePID[1],LockAnglePID[2],PIDPOS);
//     vector2fPIDInit(&trajPID,TrajPIDParam,PIDPOS);
//     vector2fPIDInit(&lockPointPID,LockPointPIDParam,PIDPOS);

//     Threshold.UseTrajThreshold = 250;
//     Threshold.TrajDoneThreshold = 70;
//     Threshold.ChassisMaxSpeed = 2.f;
// }

// void chassisLockAngle(CHASSIS *chassis,float aimAngle)
// {

//     chassis->ChassisPosSet.w = PID_Caculate(&anglePID,chassis->ChassisPosReal.angle,aimAngle);
//     if(ABS(chassis->ChassisPosSet.w) > CHASSIS_LOCK_MAX_ANGULAR_SPEED)
//         chassis->ChassisPosSet.w = GetSign(chassis->ChassisPosSet.w) * CHASSIS_LOCK_MAX_ANGULAR_SPEED;
//     if(ABS(chassis->ChassisPosSet.w) < 0.01f)
//         chassis->ChassisPosSet.w = 0;
// }
// bool chassis_LockPoint(CHASSIS* chassis,Bpoint PosSet,float AngleSet,Vector2fPID *lockpointPid,
//                         PIDType * lockanglePid)
// {

//     lockpointPid->input.x = (float)(chassis->ChassisPosReal.x);
//     lockpointPid->input.y = (float)(chassis->ChassisPosReal.y); 
//     lockpointPid->target.x = (float)PosSet.x;
//     lockpointPid->target.y = (float)PosSet.y;

//     vector2fPIDOperation(lockpointPid);

//     chassisLockAngle(chassis,AngleSet);

//     static int _lockpointCount = 0;
//     static int _lockpointTimeout = 0;

//     if(fabs(lockpointPid->err[0].x) < CHASSIS_LOCK_POS_THRESHOLD
//     && fabs(lockpointPid->err[0].y) < CHASSIS_LOCK_POS_THRESHOLD
//     && fabs(lockanglePid->err[0])  < CHASSIS_LOCK_ANGLE_THRESHOLD)
//     {
//         _lockpointTimeout = 0;
//         _lockpointCount++;
//     }
//     else
//     {
//         chassis->ChassisPosSet.vx = 
//             lockpointPid->output.x * cosf(DEG2RAD(chassis->ChassisPosReal.angle)) + 
//             lockpointPid->output.y * sinf(DEG2RAD(chassis->ChassisPosReal.angle));
//         chassis->ChassisPosSet.vy = 
//             lockpointPid->output.y * cosf(DEG2RAD(chassis->ChassisPosReal.angle)) - 
//             lockpointPid->output.x * sinf(DEG2RAD(chassis->ChassisPosReal.angle));
//         chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx,chassis->ChassisPosSet.vy});

//         if(ABS(chassis->ChassisPosSet.v) > CHASSIS_LOCK_MAX_SPEED)
//         {
//             chassis->ChassisPosSet.vx *= (CHASSIS_LOCK_MAX_SPEED / chassis->ChassisPosSet.v);
//             chassis->ChassisPosSet.vy *= (CHASSIS_LOCK_MAX_SPEED / chassis->ChassisPosSet.v);
//             chassis->ChassisPosSet.v = GetSign(chassis->ChassisPosSet.v) * CHASSIS_LOCK_MAX_SPEED;
//         }
//     }
//     if(_lockpointTimeout++ > 500)
//     {
//         _lockpointTimeout = 0;
//         return false;
//     }
//     if(_lockpointCount > 5)
//     {
//         _lockpointCount = 0;
//         chassis->crossBrake = 1;
//         BEEP_Alarm(2);
//         chassis->TurnForNextRunResearch = 0;
//         chassis->LockPoint = 0;
//         chassis->IsRunningTraj = 0;

//     }
//     return true;
// }
// void chassis_AimPoint(CHASSIS *chassis,Bpoint PosSet,PIDType *lockanglePid)
// {
//     volatile float aim_angle = 0;
//     vector2d aim_pos;
//     aim_pos.x = (float)(PosSet.x - chassis->ChassisPosReal.x);
//     aim_pos.y = (float)(PosSet.y - chassis->ChassisPosReal.y);
//     if(Modulo2d(aim_pos) < 1000.f)
//         return;
//     aim_angle = atan2f(aim_pos.y,aim_pos.x);
    
//     chassisLockAngle(chassis,RAD2DEG(aim_angle));
// }
