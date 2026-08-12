#include "myotask.h"
#include "Match.h"

void StartDefaultTask(void *argument)
{
    for (;;)
    {
        LED_Flow();
    }
}
void TaskUart(void *argument)
{
    for (;;)
    {
        Debug_ProcessRxMsg(&Chassis, &DebugRxPack);
        Debug_ProcessTxMsg(&Chassis, &DebugTxPack);
        // LaserRelocation(&SensorUsart_Msg, &Sensor_TxPack, 1, (float)testPoint.x, (float)testPoint.y, testangle);
        osDelay(10);
    }
}
void TaskChassis(void *argument)
{
    for (;;)
    {
        chassis_SensordataHandle(&Chassis, &Sensor_RxPack, &SensorUsart_Msg);
        if (Chassis.Enable)
        {
            if (Chassis.Status != CHASSIS_CLIMBOVER)
            {
                if (Chassis.LockPoint)
                    chassis_LockPoint(&Chassis, testPoint, testangle, lockPointPID + testLockPID, &anglePID, LockPointThreshold + testThrehold);
                else if (Chassis.CamLockPoint)
                {

                    if(Camera_SearchR1(&Sekiro, &Chassis))
                        Chassis.CamLockPoint = 0;
                    // Camera_GetKFS(&CameraRxPack, &Chassis, &Sekiro);
                    // Camera_LockPoint(&Chassis, (Bpoint){(s16)Sekiro.KFS_front.pos.x, (s16)Sekiro.KFS_front.pos.y, 0.f},
                    //                  CamDepth, testangle, &anglePID, lockPointPID + HarderPID, LockPointThreshold);
                }
                else if (Chassis.slopeover)
                    chassis_SlopeClimb(&Chassis);
            }
            else if (Chassis.Status == CHASSIS_CLIMBOVER)
            {
                if (Chassis.climbover)
                    chassis_ClimbPosture(&Chassis, testangle);

            }
            // else
            // {
            //     sendCarVel((s16)(Chassis.ChassisPosSet.vx * 1000.0f),(s16)(Chassis.ChassisPosSet.vy * 1000.f),
            //     (s16)(Chassis.ChassisPosSet.w * 100.f),Chassis.crossBrake);
            // }
        }
        osDelay(2);
    }
}
void Testtask(void *argument)
{
    for (;;)
    {
        sendChassisAskMsg(MASTER_CHASSIS_ASK_STEER_ANGLE);
        Match_PosUpdate(&Sekiro, &CameraRxPack, &Chassis, 0);

        osDelay(10);
    }
}

void TaskTrajctory(void *argument)
{
    for (;;)
    {
        if (Chassis.IsRunningTraj)
        {
            TrajRun(&Chassis, Trajhandler + trajMarker, &trajPID, &KeepanglePID, lockPointPID + HarderPID, LockPointThreshold + 2);
        }
        else
            osDelay(2);
    }
}

void SekiroTask(void *argument)
{
    for (;;)
    {
		TickType_t last_wake_time = osKernelGetTickCount();
        if (Sekiro.MatchStart)
            Sekiro_ShinobiExecution(&Sekiro);
        else if (Sekiro.SkillMatchStart)
            Sekiro_ShadowDiesTwice(&Sekiro);
        else if (Sekiro.half_auto || Sekiro.automatic)
        {
            if (Sekiro.Club_only)
            {
                if (Sekiro_MartialClub(&Sekiro) == TASK_FINISH)
                    Sekiro.Club_only = 0;
            }
            if (Sekiro.MFcross_only)
            {
                if (Sekiro_MFCross(&Sekiro) == TASK_FINISH)
                {
                    Sekiro.MFcross_only = 0;
                    Sekiro.state_pre = ROBOT_WAITING;
                }
            }
            if (Sekiro.Square9_only)
            {
                if (Sekiro_Tictactoe(&Sekiro) == TASK_FINISH)
                    Sekiro.Square9_only = 0;
            }
        }
        osDelay(2);
    }
}
