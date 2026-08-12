#include "chassisPid.h"

float LockAnglePID[3] = {1.65f, 0.35f, 0.01f};
float TrajPIDParam[3] = {0.0016f, 0.00035f, 0.00001f};
float LockPointPIDParam[3] = {0.0011f, 0.00035f, 0.00001f};
float LockPointPIDParamM[3] = {0.0014, 0.00035, 0.00001f};
float LockPointPIDParamH[3] = {0.0022f, 0.00005f, 0.0005f};
PIDType climbanglePID;
PIDType anglePID;
PIDType KeepanglePID;
Vector2fPID trajPID;
Vector2fPID lockPointPID[THRESHOLD_NUM];

LockangleVelThreholdTypedef LockangleVelThreshold[THRESHOLD_NUM];
LockpointThresholdTypedef LockPointThreshold[THRESHOLD_NUM];

void chassisRunInit(CHASSIS *chassis)
{
    PID_Init(&anglePID, LockAnglePID[0], LockAnglePID[1], LockAnglePID[2], PIDPOS);
    PID_Init(&climbanglePID, 2.3f, 0.f, 0.01f, PIDPOS);
    PID_Init(&KeepanglePID,1.8f,0.35f,0.01f,PIDPOS);
    PID_Init(&chassis->climb_pid,0.003,0,0.0001,PIDPOS);
    vector2fPIDInit(&trajPID, TrajPIDParam, PIDPOS);
    vector2fPIDInit(lockPointPID + NormalPID, LockPointPIDParam, PIDPOS);
    vector2fPIDInit(lockPointPID + HarderPID, LockPointPIDParamH, PIDPOS);
    vector2fPIDInit(lockPointPID + MediumHard, LockPointPIDParamM, PIDPOS);

    chassis_LockPoint_ThresholdInit(chassis, 0, 2.f, 0.08f, 6, 0.5f, 800);
    chassis_LockPoint_ThresholdInit(chassis, 1, 1.6f, 0.1f, 5, 0.4f, 350);
    chassis_LockPoint_ThresholdInit(chassis, 2, 2.0f,0.08f, 8,0.5f, 300);
}

void chassis_LockPoint_ThresholdInit(CHASSIS *chassis, uint8_t thresholdNum, float velmax, float velmin, float posthrehold, float anglethrehold, uint16_t TimeoutCount)
{
    LockPointThreshold[thresholdNum].lockposthreshold = posthrehold;
    LockPointThreshold[thresholdNum].lockpointvelmax = velmax; ///*
    LockPointThreshold[thresholdNum].lockpointvelmin = velmin;
    LockPointThreshold[thresholdNum].lockanglethreshold = anglethrehold;
    LockPointThreshold[thresholdNum].lockpointTimeout = TimeoutCount;
}

void chassisLockAngle(CHASSIS *chassis, float aimAngle)
{
    float angleDiff = aimAngle - chassis->ChassisPosReal.angle;
    angleDiff = angleDiff - 360.0f * floorf(angleDiff / 360.0f + 0.5f);
    float targetAngle = chassis->ChassisPosReal.angle + angleDiff;

    chassis->ChassisPosSet.w = PID_Caculate(&anglePID, chassis->ChassisPosReal.angle, targetAngle);
    if (ABS(chassis->ChassisPosSet.w) > CHASSIS_LOCK_MAX_ANGULAR_SPEED)
        chassis->ChassisPosSet.w = GetSign(chassis->ChassisPosSet.w) * CHASSIS_LOCK_MAX_ANGULAR_SPEED;
    if (ABS(chassis->ChassisPosSet.w) < 0.1f || fabs(angleDiff) < 0.1f)
        chassis->ChassisPosSet.w = 0;
}
bool chassis_LockPoint(CHASSIS *chassis, Bpoint PosSet, float AngleSet, Vector2fPID *lockpointPid,
                       PIDType *lockanglePid, LockpointThresholdTypedef *lockpointThreshold)
{

    lockpointPid->input.x = (float)(chassis->ChassisPosReal.x);
    lockpointPid->input.y = (float)(chassis->ChassisPosReal.y);
    lockpointPid->target.x = (float)PosSet.x;
    lockpointPid->target.y = (float)PosSet.y;

    vector2fPIDOperation(lockpointPid);

    chassisLockAngle(chassis, AngleSet);

    static int _lockpointCount = 0;
    static int _lockpointTimeout = 0;
    static int _lockpointLoseCount = 0;
    if (fabs(lockpointPid->err[0].x) < lockpointThreshold->lockposthreshold && fabs(lockpointPid->err[0].y) < lockpointThreshold->lockposthreshold && fabs(lockanglePid->err[0]) < lockpointThreshold->lockanglethreshold)
    {
        _lockpointTimeout = 0;
        _lockpointCount++;
    }
    if (_lockpointCount > 5)
    {
        _lockpointCount = 0;
        _lockpointLoseCount = 0;
        _lockpointTimeout = 0;
        chassis->TurnForNextRunResearch = 0;
        chassis->LockPoint = 0;
        chassis->IsRunningTraj = 0;
        for (int i = 0; i < 3; i++)
        {
            lockpointPid->err[i].x = 0;
            lockpointPid->err[i].y = 0;
        }
        sendCarVel(0, 0, 0, RUN_CROSSBRAKE);
        BEEP_Alarm(1);
        return true;
    }
    else
    {
        if ((fabs(lockpointPid->err[0].x) > 3.5f * lockpointThreshold->lockposthreshold || fabs(lockpointPid->err[0].y) > 3.5f * lockpointThreshold->lockposthreshold || fabs(lockanglePid->err[0]) > 2.f * lockpointThreshold->lockanglethreshold) && _lockpointCount)
        {
            _lockpointCount--;
        }
        chassis->ChassisPosSet.vx =
            lockpointPid->output.x * cosf(DEG2RAD(chassis->ChassisPosReal.angle)) +
            lockpointPid->output.y * sinf(DEG2RAD(chassis->ChassisPosReal.angle));
        chassis->ChassisPosSet.vy =
            lockpointPid->output.y * cosf(DEG2RAD(chassis->ChassisPosReal.angle)) -
            lockpointPid->output.x * sinf(DEG2RAD(chassis->ChassisPosReal.angle));
        chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx, chassis->ChassisPosSet.vy});

        if (ABS(chassis->ChassisPosSet.v) > CHASSIS_LOCK_MAX_SPEED)
        {
            chassis->ChassisPosSet.vx *= (CHASSIS_LOCK_MAX_SPEED / chassis->ChassisPosSet.v);
            chassis->ChassisPosSet.vy *= (CHASSIS_LOCK_MAX_SPEED / chassis->ChassisPosSet.v);
            chassis->ChassisPosSet.v = GetSign(chassis->ChassisPosSet.v) * CHASSIS_LOCK_MAX_SPEED;
        }
        if (chassis->TurnForNextRunResearch == 0 && !_lockpointLoseCount)
        {
            sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                       (s16)(chassis->ChassisPosSet.vy * 1000.f),
                       (s16)(chassis->ChassisPosSet.w * 100.f), RUN_PREFORNEXT);
        }
        else
            sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                       (s16)(chassis->ChassisPosSet.vy * 1000.f),
                       (s16)(chassis->ChassisPosSet.w * 100.f), RUN_NORMAL);
    }
    if (_lockpointTimeout++ > lockpointThreshold->lockpointTimeout) // 如果迟迟无法到达锁点位置
    {
        _lockpointTimeout = 0;
        if (++_lockpointLoseCount >= 2)
        {
            _lockpointLoseCount = 0;
            _lockpointCount = 0;
            chassis->TurnForNextRunResearch = 0;
            chassis->LockPoint = 0;
            chassis->IsRunningTraj = 0;
            sendCarVel(0, 0, 0, RUN_CROSSBRAKE);
            for (int i = 0; i < 3; i++)
            {
                lockpointPid->err[i].x = 0;
                lockpointPid->err[i].y = 0;
            }
            BEEP_Alarm(3);
            return true;
        }
    }
    return false;
}

void chassis_AimPoint(CHASSIS *chassis, Bpoint PosSet, PIDType *lockanglePid)
{
    volatile float aim_angle = 0;
    vector2d aim_pos;
    aim_pos.x = (float)(PosSet.x - chassis->ChassisPosReal.x);
    aim_pos.y = (float)(PosSet.y - chassis->ChassisPosReal.y);
    if (Modulo2d(aim_pos) < 1000.f)
        return;
    aim_angle = atan2f(aim_pos.y, aim_pos.x);

    chassisLockAngle(chassis, RAD2DEG(aim_angle));
}

void chassis_ClimbPosture(CHASSIS *chassis, float aimAngle)
{
    float angleDiff = aimAngle - chassis->ChassisPosReal.angle;
    angleDiff = angleDiff - 360.0f * floorf(angleDiff / 360.0f + 0.5f);
    float targetAngle = chassis->ChassisPosReal.angle + angleDiff;

    chassis->ChassisPosSet.w = PID_Caculate(&climbanglePID, chassis->ChassisPosReal.angle, targetAngle);
    if (ABS(chassis->ChassisPosSet.w) > CHASSIS_CLIMB_YAWCORRECT_SPEED)
        chassis->ChassisPosSet.w = GetSign(chassis->ChassisPosSet.w) * CHASSIS_CLIMB_YAWCORRECT_SPEED;
    if(!chassis->sendCorrect_w || fabs(angleDiff) > 45.f)
        chassis->ChassisPosSet.w = 0;
    chassis_climbSend(chassis);
}

bool Camera_LockPoint(CHASSIS *chassis, Bpoint PosReal, uint16_t Depth_X, float AngleSet, PIDType *lockanglePid, Vector2fPID *lockpointPid,
                      LockpointThresholdTypedef *lockpointThreshold)
{
    lockpointPid->input.x = (float)(PosReal.x);
    lockpointPid->input.y = (float)(PosReal.y);
    lockpointPid->target.x = (float)(Depth_X);
    lockpointPid->target.y = 0;

    vector2fPIDOperation(lockpointPid);
    static int _lockpointLoseCount = 0;
    static int _lockpointCount = 0;
    static int _lockpointTimeout = 0;
    chassisLockAngle(chassis, AngleSet);
    if (fabs(lockpointPid->err[0].x) < lockpointThreshold->lockposthreshold && fabs(lockpointPid->err[0].y) < lockpointThreshold->lockposthreshold && fabs(lockanglePid->err[0]) < lockpointThreshold->lockanglethreshold)
    {
        _lockpointTimeout = 0;
        _lockpointCount++;
    }
    if (_lockpointCount > 5)
    {
        _lockpointTimeout = 0;
        _lockpointCount = 0;
        _lockpointLoseCount = 0;
        chassis->TurnForNextRunResearch = 0;
        chassis->CamLockPoint = 0;
        chassis->IsRunningTraj = 0;
        sendCarVel(0, 0, 0, RUN_CROSSBRAKE);
        BEEP_Alarm(1);
        return true;
    }
    else
    {
        chassis->ChassisPosSet.vx = -lockpointPid->output.x;
        chassis->ChassisPosSet.vy = -lockpointPid->output.y;
        chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx, chassis->ChassisPosSet.vy});

        if (ABS(chassis->ChassisPosSet.v) > 0.6f)
        {
            chassis->ChassisPosSet.vx *= (0.6f / chassis->ChassisPosSet.v);
            chassis->ChassisPosSet.vy *= (0.6f / chassis->ChassisPosSet.v);
            chassis->ChassisPosSet.v = GetSign(chassis->ChassisPosSet.v) * 0.6f;
        }
        if (fabs(chassis->ChassisPosReal.angle - AngleSet) >= 0.5f && fabs(chassis->ChassisPosSet.w) > 5.f)
        {
            _lockpointTimeout = 0;
            chassis->ChassisPosSet.vx = 0;
            chassis->ChassisPosSet.vy = 0;
            chassis->ChassisPosSet.v = 0;
        }
        if (chassis->TurnForNextRunResearch == 0)
        {
            sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                       (s16)(chassis->ChassisPosSet.vy * 1000.f),
                       (s16)(chassis->ChassisPosSet.w * 100.f), RUN_PREFORNEXT);
        }
        else
            sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                       (s16)(chassis->ChassisPosSet.vy * 1000.f),
                       (s16)(chassis->ChassisPosSet.w * 100.f), RUN_NORMAL);
    }
    if (_lockpointTimeout++ > 800) // 如果迟迟无法到达锁点位置
    {
        _lockpointTimeout = 0;
        if (++_lockpointLoseCount >= 2)
        {
            _lockpointLoseCount = 0;
            chassis->TurnForNextRunResearch = 0;
            chassis->CamLockPoint = 0;
            chassis->IsRunningTraj = 0;
            sendCarVel(0, 0, 0, RUN_CROSSBRAKE);
            BEEP_Alarm(3);
        }
    }
    return false;
}
