#include "Match.h"

uint8_t MatchPhase = 0;

void Match_Init(SEKIRO *sekiro)
{
    static u16 pause_count = 0;
    if (Chassis.Enable && Actparam.enable)
    {
        if (sekiro->task == TASK_INIT)
        {
            sendCarVel(0, 0, 0, RUN_CROSSBRAKE);
            // Actuator_WarheadReady(&Actparam, Sekiro.side);
            sekiro->task = TASK_PROCESS;
        }
        else if (sekiro->task == TASK_PROCESS)
        {
            if (pause_count++ > 50)
            {
                sekiro->state = ROBOT_WAITING;
                sekiro->task = TASK_INIT;
                pause_count = 0;
                MatchPhase++;
                WarheadFetch_Num = 1;
                if(sekiro->Zone2Restart)
                {
                    MatchPhase = MatchZone1;
                    sekiro->state = ROBOT_WAITING;
                    Actparam.warhead_docked = WarheadFetch_Num;
                    sekiro->state_pre = ROBOT_DOCKING_WARHEAD;
										sekiro->Docked = 1;
                    sekiro->LeaveZone1 = 1;
                    sekiro->offpath = sekiro->retry_path;
                    KFSDiscardDicide(sekiro);
                    RobotCom_ToActuator(&RobotRxmsg);
                }
                if (sekiro->Zone3Restart)
                {
                    MatchPhase = MatchZone3;
                    sekiro->state = ROBOT_WAITING;
                    sekiro->state_pre = ROBOT_WAITING;
                    sekiro->Slope_Climb = true;
                }
                if (sekiro->SkillMatchStart && SkillMatchType == Nine_Palace)
                {
                    MatchPhase = MatchZone3;
                    sekiro->state = ROBOT_CLIMBSLOPE;
                    sekiro->state_pre = ROBOT_WAITING;
                }
                else if(sekiro->SkillMatchStart && SkillMatchType == Martial_Mysteries)
                {
                    WarheadFetch_Num = 3;
                    if(sekiro->side == RED)
                    {
                        Actparam.warhead_num[0] = 0;
                        Actparam.warhead_num[1] = 1;
                        Actparam.warhead_num[2] = 2;
                        
                    }
                    else if(sekiro->side == BLUE)
                    {
                        Actparam.warhead_num[0] = 3;
                        Actparam.warhead_num[1] = 4;
                        Actparam.warhead_num[2] = 5;
                    }
                }
            }
        }
    }
}

Task_State Sekiro_MartialClub(SEKIRO *sekiro)
{
    switch (sekiro->state)
    {
    case ROBOT_WAITING:
        if (sekiro->automatic || (sekiro->half_auto && sekiro->Nxt_move))
        {
            sekiro->task = TASK_INIT;
            switch (sekiro->state_pre)
            {
            case ROBOT_CHASSISRUN:
                if (Actparam.warhead_docked >= WarheadFetch_Num)
                {
                    Actparam.warhead_docked = 0;
                    return TASK_FINISH;
                }
                else
                    sekiro->state = ROBOT_GRABBING_WARHEAD;
                break;
            case ROBOT_GRABBING_WARHEAD:
                sekiro->state = ROBOT_DOCKING_WARHEAD;
                break;
            case ROBOT_DOCKING_WARHEAD:
                sekiro->state = ROBOT_CHASSISRUN;
                break;
            case ROBOT_WAITING:
                sekiro->state = ROBOT_CHASSISRUN;
                break;
            case ROBOT_BACKTOEDGE:
                break;
            default:
                break;
            }
        }
        break;
    case ROBOT_GRABBING_WARHEAD:
        Match_GrabWarhead(sekiro, &Actparam);
        break;
    case ROBOT_DOCKING_WARHEAD:
        Match_WarheadDocking(sekiro, &Actparam, &Chassis);
        break;
    case ROBOT_CHASSISRUN:
        if (Actparam.warhead_docked < WarheadFetch_Num)
            Match_Chassis2Warhead(sekiro, &Actparam, &Chassis);
        else
            Match_ChassisToTheEdge(sekiro, &Chassis);
        break;
    default:
        break;
    }
    return TASK_PROCESS;
}
Task_State Sekiro_MFCross(SEKIRO *sekiro) // 状态机
{
    bool height = 0;
    //    if(sekiro->current_zone == ZONE2)
    //    {
    switch (sekiro->state)
    {
    case ROBOT_WAITING: // 中转状态
        if (sekiro->automatic || (sekiro->half_auto && sekiro->Nxt_move))
        {
            if (sekiro->state_pre == ROBOT_ACCENDING || sekiro->state_pre == ROBOT_DESCENDING)
            {

                if (Match_PosUpdate(sekiro, &CameraRxPack, &Chassis, 1) == TASK_FINISH)
                {
                    if (sekiro->zone2_field == 13)
                    {
                        testLockPID = NormalPID;
                        testThrehold = 0;
                        return TASK_FINISH;
                    }
                    else
                    {
                        // TODO 我把判断是否要旁取给到上台阶之后, 判断成功就直接不进ROBOT_CHASISRUN 就不在ChassisPosAdapt里调整了,代码会简洁一些
                        //   本身Match_GrabKFS也写了底盘调整 --wyh
                        if (KFSGetTraversal(sekiro, &Actparam))
                            sekiro->state = ROBOT_GRABBING_KFS;
                        else
                            sekiro->state = ROBOT_CHASSISRUN;
                    }
                }
            }
            else if (sekiro->state_pre == ROBOT_CHASSISRUN)
            {

                if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS == R1_KFS &&
                    sekiro->KFS_front.type == R1_KFS)
                    sekiro->state = ROBOT_WAITING;
                else
                {
                    if (sekiro->offpath.front == 0)
                    {
                        if (sekiro->offpath.side_pick && sekiro->offpath.KFS_Pos[0] <= 3)
                            sekiro->state = ROBOT_GRABBING_KFS;
                        else
                            sekiro->state = ROBOT_ACCENDING;
                    }

                    else if (sekiro->offpath.path[sekiro->offpath.front] == 13)
                        sekiro->state = ROBOT_DESCENDING;
                    else
                    {
                        if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height >
                            sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height)
                            sekiro->state = ROBOT_ACCENDING;
                        else
                            sekiro->state = ROBOT_DESCENDING;
                    }
                }
            }
            else if (sekiro->state_pre == ROBOT_GRABBING_KFS)
            {
                if (KFSGetTraversal(sekiro, &Actparam))
                {
                    sekiro->state = ROBOT_GRABBING_KFS;
                }
                else
                    sekiro->state = ROBOT_CHASSISRUN;
            }
            else if (sekiro->state_pre == ROBOT_WAITING)
            {
                sekiro->state = ROBOT_CHASSISRUN;
            }
            sekiro->task = TASK_INIT;
        }
        break;
    case ROBOT_ACCENDING:
        if (sekiro->offpath.front == 0)
            height = (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height > 20);
        else
            height = (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height - sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height > 20) ? 1 : 0;
        if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS == R2_KFS)
        {
            if (!sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].R2_KFSDiscard)
            {
                Match_Ascend(sekiro, &Chassis, height, 1);
            }
            else
                Match_Ascend(sekiro, &Chassis, height, 2);
        } 
        else
        {
            if(sekiro->offpath.path[sekiro->offpath.front] == 1 || sekiro->offpath.path[sekiro->offpath.front] == 3)
            {
                Match_Ascend(sekiro, &Chassis, height, 3);
            }
            else
            {
                Match_Ascend(sekiro, &Chassis, height, 0);
            }
        }

        break;
    case ROBOT_DESCENDING:
        if (sekiro->offpath.path[sekiro->offpath.front] == 13)
            height = (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height == 40) ? 1 : 0;
        else
            height = (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height - sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height > 20) ? 1 : 0;
        if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS == R2_KFS && sekiro->offpath.path[sekiro->offpath.front] != 13)
        {
            if (!sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].R2_KFSDiscard)
            {
                if(sekiro->offpath.path[sekiro->offpath.front] == 4)
                    Match_Descend(sekiro, &Chassis, height, 4);
                else
                    Match_Descend(sekiro, &Chassis, height, 1);
            }
            else
                Match_Descend(sekiro, &Chassis, height, 2);
        }

        else
            Match_Descend(sekiro, &Chassis, height, 0);
        break;
    case ROBOT_GRABBING_KFS:
        if (sekiro->offpath.front == 0)
            Match_High400mmGrabKFS(sekiro, &Chassis, &Actparam);
        else
            Match_GrabKFS(sekiro, &Chassis);
        break;
    case ROBOT_CHASSISRUN:
        Match_PostureAdapt(sekiro, &Chassis);
        break;
    default:
        break;
    }
    return TASK_PROCESS;
    //    }
    //    return TASK_ERROR;
}

Task_State Sekrio_MEILINCross_NoRoutes(SEKIRO *sekiro)
{
    switch (sekiro->state)
    {
    case ROBOT_WAITING:
        if (sekiro->automatic || (sekiro->half_auto && sekiro->Nxt_move))
        {
            if (sekiro->state_pre == ROBOT_ACCENDING || sekiro->state_pre == ROBOT_DESCENDING)
            {
                if (Match_PosUpdate(sekiro, &CameraRxPack, &Chassis, 1) == TASK_FINISH)
                {
                    if (sekiro->zone2_field == 13)
                    {
                        testLockPID = NormalPID;
                        testThrehold = 0;
                        return TASK_FINISH;
                    }
                    else
                        sekiro->state = ROBOT_CHASSISRUN;
                }
            }
            else if (sekiro->state_pre == ROBOT_WAITING)
            {
                sekiro->offpath.front = 0;
                sekiro->offpath.path[0] = 2;
                for (int i = 0; i < 12; i++)
                    sekiro->Map_status.field[i].field_KFS = UNKNOWN_KFS;
                sekiro->state = ROBOT_CHASSISRUN;
            }
            else if (sekiro->state_pre == ROBOT_CHASSISRUN)
            {
                if (KFSGetTraversal(sekiro, &Actparam))
                {
                    sekiro->state = ROBOT_GRABBING_KFS;
                }
                else if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS == R1_KFS &&
                         sekiro->KFS_front.type == R1_KFS)
                    sekiro->state = ROBOT_WAITING;
                else
                {
                    if (sekiro->offpath.front == 0)
                        sekiro->state = ROBOT_ACCENDING;
                    else if (sekiro->offpath.path[sekiro->offpath.front] == 13)
                        sekiro->state = ROBOT_DESCENDING;
                    else
                    {
                        if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height >
                            sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height)
                            sekiro->state = ROBOT_ACCENDING;
                        else
                            sekiro->state = ROBOT_DESCENDING;
                    }
                }
            }
            else if (sekiro->state_pre == ROBOT_GRABBING_KFS)
            {
                sekiro->state = ROBOT_CHASSISRUN;
            }
        }
        break;
    case ROBOT_ACCENDING:
        break;
    case ROBOT_DESCENDING:
        break;
    case ROBOT_CHASSISRUN:
        break;
    case ROBOT_GRABBING_KFS:
        break;
    default:
        break;
    }
    return TASK_PROCESS;
}

Task_State Sekiro_Tictactoe(SEKIRO *sekiro)
{
    switch (sekiro->state)
    {
    case ROBOT_WAITING:
        if (sekiro->automatic || (sekiro->half_auto && sekiro->Nxt_move))
        {
            switch (sekiro->state_pre)
            {
            case ROBOT_WAITING:
                sekiro->state = ROBOT_CHASSISRUN;
                break;
            case ROBOT_CLIMBSLOPE:
                sekiro->state = ROBOT_CHASSISRUN;
                break;
            case ROBOT_CHASSISRUN:
                if (sekiro->SlopeAfter)
                {
                    sekiro->state = ROBOT_UPTOR1;
                }
                else
                    sekiro->state = ROBOT_CHASSISRUN;
                break;
            case ROBOT_UPTOR1:
                sekiro->state = ROBOT_PLACING_KFS;
                RobotRxmsg.receive_effect = true;
                break;
            case ROBOT_PLACING_KFS:
                if (sekiro->R1PlaceKFS == MIDDLEPLACE || sekiro->R1PlaceKFS == TOPPLACE )
                    sekiro->state = ROBOT_PLACING_KFS;
                else
                    sekiro->state = ROBOT_GRABR1KFS;
                break;
            default:
                break;
            }

            sekiro->task = TASK_INIT;
        }
        break;
    case ROBOT_CLIMBSLOPE:
        Match_ClimbSlope(sekiro, &Chassis);
        break;
    case ROBOT_CHASSISRUN:
        if (sekiro->Slope_Climb)
            Match_ChassisToSquare9(sekiro, &Chassis);
        else
            Match_ChassisToZone3Entrance(sekiro, &Chassis);
        break;
    case ROBOT_PLACING_KFS:
        if (sekiro->R1PlaceKFS == 1)
            Match_KFSPutMiddle(sekiro, &Chassis, &Actparam);
        else if (sekiro->R1PlaceKFS == 2 && Chassis.ChassisPosReal.z >= 1050)
            Match_KFSPutTop(sekiro, &Actparam);
        break;
    case ROBOT_UPTOR1:
        if(CameraLightPack.LightState == 0)
            Match_UptoR1(sekiro, &Chassis);
        break;
    case ROBOT_GRABR1KFS:
        Match_GrabR1KFS(sekiro, &Actparam);
        break;
    default:
        break;
    }
    return TASK_PROCESS;
}

Task_State Sekiro_SkillTictactoe(SEKIRO *sekiro)
{
    switch (sekiro->state)
    {
    case ROBOT_WAITING:
        if (sekiro->automatic || (sekiro->half_auto && sekiro->Nxt_move))
        {
            switch (sekiro->state_pre)
            {
            case ROBOT_WAITING:
                sekiro->state = ROBOT_CLIMBSLOPE;
                break;
            case ROBOT_CLIMBSLOPE:
                sekiro->state = ROBOT_CHASSISRUN;
                break;
            case ROBOT_CHASSISRUN:
                sekiro->state = ROBOT_GRABBING_KFS;
                break;
            case ROBOT_UPTOR1:
                sekiro->state = ROBOT_PLACING_KFS;
                RobotRxmsg.receive_effect = true;
                break;
            case ROBOT_GRABBING_KFS:
                if(Actparam.KFS_load == 1)
                    sekiro->state = ROBOT_CHASSISRUN;
                else if(Actparam.KFS_load == 2)
                    sekiro->state = ROBOT_UPTOR1;
                break;
            case ROBOT_PLACING_KFS:
                if (sekiro->R1PlaceKFS == MIDDLEPLACE || sekiro->R1PlaceKFS == TOPPLACE )
                    sekiro->state = ROBOT_PLACING_KFS;
                else
                    sekiro->state = ROBOT_GRABR1KFS;
                break;
            default:
                break;
            }

            sekiro->task = TASK_INIT;
        }
        break;
    case ROBOT_PLACING_KFS:
        if (sekiro->R1PlaceKFS == 1)
            Match_KFSPutMiddle(sekiro, &Chassis, &Actparam);
        else if (sekiro->R1PlaceKFS == 2 &&Chassis.ChassisPosReal.z >= 1050)
            Match_KFSPutTop(sekiro, &Actparam);
        break;
    case ROBOT_GRABBING_KFS:
        SkillMatch_GrabKFS(sekiro, &Actparam, &Chassis);
        break;
    case ROBOT_UPTOR1:
        if(CameraLightPack.LightState == 0)
            Match_UptoR1(sekiro,&Chassis);
        break;
    case ROBOT_CLIMBSLOPE:
        Match_ChassisToSquare9(sekiro,&Chassis);
        break;
    case ROBOT_CHASSISRUN:
        SkillMatch_ChassisAdapt(sekiro,&Chassis);
        break;
    default:
        break;
    }
		return TASK_PROCESS;
}
void Sekiro_ShinobiExecution(SEKIRO *sekiro)
{
    if (sekiro->automatic)
    {
        switch (MatchPhase)
        {
        case MatchInit:
            Match_Init(sekiro);
            break;
        case MatchZone1:
            if (Sekiro_MartialClub(sekiro) == TASK_FINISH)
            {
                MatchPhase++;
                sekiro->state = ROBOT_WAITING;
                sekiro->task = TASK_INIT;
            }
            break;
        case MatchZone2:
            if (Sekiro_MFCross(sekiro) == TASK_FINISH)
            {
                MatchPhase++;
                sekiro->state = ROBOT_WAITING;
                sekiro->state_pre = ROBOT_WAITING;
                sekiro->task = TASK_INIT;
            }
            break;
        case MatchZone3:
            if (Sekiro_Tictactoe(sekiro) == TASK_FINISH)
            {
                MatchPhase = 0;
                sekiro->state = ROBOT_WAITING;
                sekiro->state_pre = ROBOT_WAITING;
                sekiro->task = TASK_INIT;
            }
            break;
        default:
            break;
        }
    }
}

void Sekiro_ShadowDiesTwice(SEKIRO *sekiro)
{
    if (SkillMatchType == Martial_Mysteries)
    {
        switch (MatchPhase)
        {
        case MatchInit:
            Match_Init(sekiro);
            break;
        case MatchZone1:
            if (Sekiro_MartialClub(sekiro) == TASK_FINISH)
            {
                    MatchPhase++;
                    sekiro->state = ROBOT_WAITING;
                    sekiro->task = TASK_INIT;
            }
            break;
        case MatchZone2:
            if (Sekiro_MFCross(sekiro) == TASK_FINISH)
            {
                MatchPhase++;
                sekiro->state = ROBOT_WAITING;
                sekiro->task = TASK_INIT;
            }
            break;
        default:
            break;
        }
    }
    else if(SkillMatchType == Nine_Palace)
    {
        if (MatchPhase == MatchInit)
        {
            Match_Init(sekiro);
            SensorUsart_Msg.offset_x = 6700;
            SensorUsart_Msg.offset_y = 4600 * (sekiro->side == BLUE ? 1 : -1);
        }
        else if (MatchPhase == MatchZone3)
        {
            if (Sekiro_SkillTictactoe(sekiro) == TASK_FINISH)
            {
                MatchPhase = 0;
                sekiro->state = ROBOT_WAITING;
                sekiro->state_pre = ROBOT_WAITING;
                sekiro->task = TASK_INIT;
            }
        }
    }

}


