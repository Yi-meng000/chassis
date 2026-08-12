#include "Trajset.h"

TrajCommonParam trajArray[15];
TrajParam Trajhandler[15];
Bpoint PointNow, PointAim, PointGoal;
float delta_T = 0.01f;
float AngleNow = 0, AngleAim = 0;
float Trajectory_SpeedUp(float tbegin, float tnow, float tup, float vel_start, float vel_end)
{
    float vel_now = 0;
    if (tnow > tbegin && tnow < (tup + tbegin))
    {
        vel_now = vel_start + (vel_end - vel_start) * sqrtf((tnow - tbegin) / tup);
    }
    else
        vel_now = vel_end;
    return vel_now;
}
float Trajectory_SpeedDown(float tbegin, float tnow, float tdown, float vel_start, float vel_end)
{
    float vel_now = 0;

    if (tbegin + tdown > 1)
        tbegin = 1 - tdown;
    if (tnow > tbegin && tnow < (tdown + tbegin))
        vel_now = vel_end + (vel_start - vel_end) * sqrtf((tbegin + tdown - tnow) / tdown);
    else
        vel_now = vel_end;

    return vel_now;
}

float TRAJSET_SinSpeed(float T, float tnow, float velstart, float velend)
{
    if (tnow < 0)
        return velstart;
    if (tnow > T)
        return velend;
    float k = sinf(PI * tnow / T - PI / 2);
    return (velstart + 0.5f * (k + 1) * (velend - velstart));
}

float Trajectory_Rotate(TrajParam *traj_param)
{
		uint32_t TimeNow = HAL_GetTick() - traj_param->start_time;
    float rotateTnow = (float)TimeNow / 1000.f / traj_param->TotalT - traj_param->rotateBeginT;
    float angleTemp = 0;
    if (rotateTnow > traj_param->rotateNeedT)
        angleTemp = traj_param->endAngle;
    else if (rotateTnow > 0 && rotateTnow < traj_param->rotateNeedT)
        angleTemp = traj_param->startAngle + (traj_param->endAngle - traj_param->startAngle) * rotateTnow / traj_param->rotateNeedT;
    else
        angleTemp = traj_param->startAngle;
    return angleTemp;
}
void TrajParam_SetPoints(TrajName index, uint8_t trajmode, uint8_t rank,
                         vector2d point[],
                         float uniformSpeed, float start_angle, float end_angle,
                         uint8_t ifLockPointAfterTraj)
{
    trajArray[index].trajMode = trajmode;
    trajArray[index].rank = rank;
    for (int i = 0; i < rank + 1; i++)
    {
        trajArray[index].CtrlPoints[i].x = (s16)point[i].x;
        trajArray[index].CtrlPoints[i].y = (s16)point[i].y;
    }
    trajArray[index].uniformSpeed = uniformSpeed;
    trajArray[index].startangle = start_angle;
    trajArray[index].endangle = end_angle;
    trajArray[index].iflockpointafterTraj = ifLockPointAfterTraj;
}

void BezierParam_Init(TrajParam *param, uint8_t index, uint8_t speedmode, uint8_t anglemode,
                      float startspeed, float endspeed, float speedupS, float speeddownS,
                      float rotatebeginT, float rotateneedT, float usetrajthrehold,
                      float trajdonethreshold)
{
    param->trajMode = trajArray[index].trajMode;
    param->rank = trajArray[index].rank;
    param->CtrlPoints = trajArray[index].CtrlPoints;
    param->length = BezierLengthGauss(param->CtrlPoints, param->rank, 50);

    param->uniformSpeed = trajArray[index].uniformSpeed;
    param->startSpeed = startspeed;
    param->endSpeed = endspeed;
    param->startAngle = trajArray[index].startangle;
    param->endAngle = trajArray[index].endangle;

    param->ifLockPointAfterTraj = trajArray[index].iflockpointafterTraj;
    param->trajSpeedMode = speedmode;
    param->trajAngleMode = anglemode;

    param->SpeedUpS = speedupS;
    param->SpeedDownS = speeddownS;
    param->SpeedUniformS = 1 - param->SpeedUpS - param->SpeedDownS;

    param->speedUpT = param->length * param->SpeedUpS * 2 / (param->uniformSpeed + param->startSpeed) / 1000.f;
    param->speedDownT = param->length * param->SpeedDownS * 2 / (param->uniformSpeed + param->endSpeed) / 1000.f;
    param->speedUniformT = param->length * param->SpeedUniformS / param->uniformSpeed / 1000.f;
    param->TotalT = param->speedUpT + param->speedDownT + param->speedUniformT;

    param->rotateBeginT = rotatebeginT;
    param->rotateNeedT = rotateneedT;

    param->trajthreshold.UseTrajThreshold = usetrajthrehold;
    param->trajthreshold.TrajDoneThreshold = trajdonethreshold;
    param->trajthreshold.ChassisMaxSpeed = CHASSIS_MAX_VELOCITY;
    param->chasepoint1.t = 0;
    param->TrajRunInit = 0;
    param->Inaroll = 0;
}

void Trajectory_OffPointSet(TrajParam *param, s16 offset_x, s16 offset_y)
{
		param->Point_offset.x = offset_x;
		param->Point_offset.y = offset_y;
}	
float GetRatio(TrajParam *param, float runtime)
{
    float length_m = param->length / 1000.f; /* length 单位 mm -> m */
    float s = 0;
 
    if (runtime <= 0)
        return 0.f;
    if (runtime >= param->TotalT)
        return 1.f;
 
    if (runtime <= param->speedUpT)
    {
        /* 正弦加速段积分:∫v dt = v0*t + (v1-v0)*(t/2 - T/(2π)·sin(πt/T)) */
        s = param->startSpeed * runtime +
            (param->uniformSpeed - param->startSpeed) *
                (runtime / 2.f - param->speedUpT / (2.f * PI) * sinf(PI * runtime / param->speedUpT));
    }
    else if (runtime <= param->speedUpT + param->speedUniformT)
    {
        s = length_m * param->SpeedUpS +
            param->uniformSpeed * (runtime - param->speedUpT);
    }
    else
    {
        float t = (param->TotalT - runtime);
        float T = param->speedDownT;
        float s_remain = param->endSpeed * t +
                         (param->uniformSpeed - param->endSpeed) *
                             (t / 2.f - T / (2.f * PI) * sinf(PI * t / T));
        s = length_m - s_remain;
    }
 
    float ratio = s / length_m;
    if (ratio < 0.f)
        ratio = 0.f;
    if (ratio > 1.f)
        ratio = 1.f;
    return ratio;
}

float GetBezierT(TrajParam *param, float runtime)
{
    return BezierGetParamT(param->CtrlPoints, param->rank, GetRatio(param, runtime) * param->length);
}

bool TrajRun(CHASSIS *chassis, TrajParam *trajparam, Vector2fPID *trajpid, PIDType *lockanglepid, Vector2fPID *lockpointpid, LockpointThresholdTypedef *lockpointThreshold)
{
    vector2d velTrajReal, velTrajAim, velCtrl, pidOutput, poserrOutput;
    Bpoint _chasepoint1, _chasepoint2, posAimNow, traj_lock = {0};
    float projectTrajReal, angletemp;
    uint32_t TimeNow = 0;

    PointNow.x = chassis->ChassisPosReal.x;
    PointNow.y = chassis->ChassisPosReal.y;
    AngleNow = chassis->ChassisPosReal.angle;

    if (!trajparam->TrajRunInit)
    {
        BezierLengthTableUpdate(trajparam->CtrlPoints, trajparam->rank);
        trajparam->start_time = HAL_GetTick();
        trajparam->TrajRunInit = true;
    }

    delta_T = 0.1f;
    static TickType_t xFrequency = pdMS_TO_TICKS(TRAJ_PERIOD);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float trajdonelength = Modulo2d(Vector_Point2vector(PointNow, trajparam->CtrlPoints[trajparam->rank]));

    if (trajparam->chasepoint1.t < 0)
        trajparam->chasepoint1.t = 0;
    else if (trajparam->chasepoint1.t > trajparam->TotalT)
        trajparam->chasepoint1.t = trajparam->TotalT;
    if (trajparam->chasepoint1.t <= (trajparam->TotalT - delta_T) && trajdonelength > trajparam->trajthreshold.TrajDoneThreshold)
    {

        trajparam->chasepoint2.t = trajparam->chasepoint1.t + delta_T;

        switch (trajparam->trajMode)
        {
        case Bezier:
            _chasepoint1 = calBezierPoint(trajparam->CtrlPoints,
                                          trajparam->rank, GetBezierT(trajparam, trajparam->chasepoint1.t));
            _chasepoint2 = calBezierPoint(trajparam->CtrlPoints,
                                          trajparam->rank, GetBezierT(trajparam, trajparam->chasepoint2.t));
            break;
        case Bspline5:
            break;
        default:
            break;
        }

        trajparam->chasepoint1.x = _chasepoint1.x;
        trajparam->chasepoint1.y = _chasepoint1.y;
        trajparam->chasepoint2.x = _chasepoint2.x;
        trajparam->chasepoint2.y = _chasepoint2.y;

        chassis->ChassisPosSet.x = _chasepoint2.x;
        chassis->ChassisPosSet.y = _chasepoint2.y;
        velTrajReal = Vector_Point2vector(trajparam->chasepoint1, PointNow);
        velTrajAim = Vector_Point2vector(trajparam->chasepoint1, trajparam->chasepoint2);
        if (Modulo2d(velTrajAim) > 1e-5f)
            velTrajAim = Vector_MultiplyNum(velTrajAim, (1.f / Modulo2d(velTrajAim)));

        switch (trajparam->trajSpeedMode)
        {
        case Square:
            if (trajparam->chasepoint2.t < trajparam->speedUpT)
            {
                velCtrl = Vector_MultiplyNum(
                    velTrajAim, Trajectory_SpeedUp(0, trajparam->chasepoint2.t,
                                                   trajparam->speedUpT, trajparam->startSpeed,
                                                   trajparam->uniformSpeed));
            }
            else if ((trajparam->TotalT - trajparam->chasepoint2.t) < trajparam->speedDownT)
            {
                velCtrl = Vector_MultiplyNum(
                    velTrajAim, Trajectory_SpeedDown(trajparam->TotalT - trajparam->speedDownT, trajparam->chasepoint2.t,
                                                     trajparam->speedDownT, trajparam->uniformSpeed,
                                                     trajparam->endSpeed));
            }
            else
            {
                velCtrl =
                    Vector_MultiplyNum(velTrajAim, trajparam->uniformSpeed);
            }
            break;
        case SinF:
            if (trajparam->chasepoint2.t <= trajparam->speedUpT)
                velCtrl = Vector_MultiplyNum(
                    velTrajAim, TRAJSET_SinSpeed(trajparam->speedUpT, trajparam->chasepoint2.t,
                                                 trajparam->startSpeed, trajparam->uniformSpeed));
            else if ((trajparam->TotalT - trajparam->chasepoint2.t) < trajparam->speedDownT)
            {
                velCtrl = Vector_MultiplyNum(
                    velTrajAim, TRAJSET_SinSpeed(trajparam->speedDownT,
                                                 trajparam->chasepoint2.t - (trajparam->TotalT - trajparam->speedDownT),
                                                 trajparam->uniformSpeed,
                                                 trajparam->endSpeed));
            }
            else
            {
                velCtrl = Vector_MultiplyNum(velTrajAim, trajparam->uniformSpeed);
            }
            break;
        case SelfDefine:
            break;
        default:
            break;
        }

        projectTrajReal = Vector_InnerProduct(velTrajReal, velTrajAim);

        velTrajAim = Vector_MultiplyNum(velTrajAim, projectTrajReal);

        TimeNow = HAL_GetTick() - trajparam->start_time;

        trajpid->input = velTrajReal;
        trajpid->target = velTrajAim;
        pidOutput = vector2fPIDOperation(trajpid);
        if (Modulo2d(trajpid->output) > (trajparam->uniformSpeed ))
        {
            pidOutput = Vector_MultiplyNum(trajpid->output, trajparam->uniformSpeed  / Modulo2d(trajpid->output));
        }

        float velmod = Modulo2d(velCtrl);
				
        posAimNow = calBezierPoint(trajparam->CtrlPoints, trajparam->rank, GetBezierT(trajparam, (float)(TimeNow) / 1000.f));
        lockpointpid->input.x = chassis->ChassisPosReal.x;
        lockpointpid->input.y = chassis->ChassisPosReal.y;
        lockpointpid->target.x = posAimNow.x;
        lockpointpid->target.y = posAimNow.y;
        poserrOutput = vector2fPIDOperation(lockpointpid);

        if (Modulo2d(poserrOutput) >  velmod * 0.1f)
        {
            poserrOutput = Vector_MultiplyNum(poserrOutput, velmod * 0.1f / Modulo2d(poserrOutput));
        }
		velCtrl.x += pidOutput.x + poserrOutput.x;
        velCtrl.y += pidOutput.y + poserrOutput.y;
        switch (trajparam->trajAngleMode)
        {
        case Spin:
            angletemp = Trajectory_Rotate(trajparam);
            break;
        case Follow:
            break;
        default:
            break;
        }
        chassis->ChassisPosSet.angle = angletemp;
        chassisLockAngle(chassis, angletemp);

        chassis->ChassisPosSet.vx = velCtrl.x * cosf(DEG2RAD(chassis->ChassisPosReal.angle)) + velCtrl.y * sinf(DEG2RAD(chassis->ChassisPosReal.angle));
        chassis->ChassisPosSet.vy = velCtrl.y * cosf(DEG2RAD(chassis->ChassisPosReal.angle)) - velCtrl.x * sinf(DEG2RAD(chassis->ChassisPosReal.angle));

        chassis->ChassisPosSet.v = Modulo2d((vector2d){chassis->ChassisPosSet.vx, chassis->ChassisPosSet.vy});

        if (chassis->ChassisPosSet.v > trajparam->trajthreshold.ChassisMaxSpeed)
        {
            chassis->ChassisPosSet.vx = chassis->ChassisPosSet.vx * trajparam->trajthreshold.ChassisMaxSpeed / chassis->ChassisPosSet.v;
            chassis->ChassisPosSet.vy = chassis->ChassisPosSet.vy * trajparam->trajthreshold.ChassisMaxSpeed / chassis->ChassisPosSet.v;
            chassis->ChassisPosSet.v = trajparam->trajthreshold.ChassisMaxSpeed * GetSign(chassis->ChassisPosSet.v);
        }
        if (chassis->TurnForNextRunResearch == 0)
        {
            sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                       (s16)(chassis->ChassisPosSet.vy * 1000.f),
                       (s16)(chassis->ChassisPosSet.w * 100.f), RUN_PREFORNEXT);
            if (TimeNow >= 500)
            {
                chassis->TurnForNextRunResearch = 1;
            }
        }
        else
            sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                       (s16)(chassis->ChassisPosSet.vy * 1000.f),
                       (s16)(chassis->ChassisPosSet.w * 100.f), RUN_NORMAL);
        if (Modulo2d(velTrajAim) > 1e-4f)
        {
            float plus_t = projectTrajReal * delta_T /
                    Modulo2d(Vector_Point2vector(trajparam->chasepoint1, trajparam->chasepoint2));
            if(plus_t < 0)
                plus_t = 0;
            if(plus_t > 0.1f)
                plus_t = 0.1f;
            if(Modulo2d(Vector_Point2vector(trajparam->chasepoint1, trajparam->chasepoint2)) > 1.f)
                trajparam->chasepoint1.t += plus_t;
            else
                trajparam->chasepoint1.t += delta_T;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        return 0;
    }
    else
    {
        if (trajparam->ifLockPointAfterTraj == LockPoint_Brake)
        {
            chassis->TurnForNextRunResearch = 0;
						traj_lock.x = trajparam->CtrlPoints[trajparam->rank].x + trajparam->Point_offset.x;
						traj_lock.y = trajparam->CtrlPoints[trajparam->rank].y + trajparam->Point_offset.y;
						
            while (!chassis_LockPoint(chassis, traj_lock,
                                      trajparam->endAngle, lockPointPID, &anglePID, lockpointThreshold))
            {
                osDelay(2);
            }

            BEEP_Alarm(1);
        }
        else if (trajparam->ifLockPointAfterTraj == Cross_Brake)
        {
            BEEP_Alarm(2);
            sendCarVel(0, 0, 0, RUN_CROSSBRAKE);
            // TODO crossLock(chassis);
        }
        else
        {
            BEEP_Alarm(2);
            if(!trajparam->Inaroll)
                sendCarVel(0, 0, 0, RUN_NORMAL);
        }
        trajparam->chasepoint1.t = 0;
        trajparam->TrajRunInit = 0;
        chassis->IsRunningTraj = 0;
        chassis->TurnForNextRunResearch = 0;
        for (int i = 0; i < 3; i++)
        {
            trajpid->err[i].x = 0;
            trajpid->err[i].y = 0;
            lockpointpid->err[i].x = 0;
            lockpointpid->err[i].y = 0;
        }

        return 1;
    }
}
