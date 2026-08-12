#include "chassisRun.h"

uint8_t trajMarker = 0;
bool *side_traj = NULL;
void TrajctoryParam_Init(void)
{
    vector2d TestTrajPoint[6] = {{0, 0}, {115.2, 255}, {288, 555}, {320, 750}, {480, 905}, {645, 910}};
    TrajParam_SetPoints(TestTraj, Bezier, 5, TestTrajPoint, 1.f, -90.f, -90.f, LockPoint_Brake);
    BezierParam_Init(Trajhandler + TestTraj, TestTraj, SinF, Spin,
                     0.3f, 0.0f, 0.1f, 0.7f, 0.f, 0.0, 50, 10);

    vector2d ToWarhead[6] = {{0, 0}, {0, -630}, {80, -900}, {60, -900}, {150, -900}, {240, -900}};
    TrajParam_SetPoints(Zone1_ToWarehead, Bezier, 5, ToWarhead, 0.8, 0, 0, Cross_Brake);
    BezierParam_Init(Trajhandler + Zone1_ToWarehead, Zone1_ToWarehead, SinF, Spin,
                     0.2f, 0, 0.2f, 0.8f, 0, 0, 0.8f, 10);

    vector2d AfterSlope[6] = {{11030, 3885}, {10900, 2272}, {10700, 1972}, {10600, 1670}, {10450, 1200}, {10350, 1080}};
    TrajParam_SetPoints(Zone3_AfterSlope, Bezier, 5, AfterSlope, 2.0f, -90.f, -90.f , LockPoint_Brake);
    BezierParam_Init(Trajhandler + Zone3_AfterSlope, Zone3_AfterSlope, SinF, Spin,
                     1.8f, 0.f, 0.1f, 0.5f, 0.15f, 0.30f, 120, 50);
			
    vector2d Z1toZ2_3[6] = {    {460,-400},
                                {900,0},
                                {1350,360},
                                {1680,460},
                                {1940,435},
                                {2130,430}};
    TrajParam_SetPoints(Zone1EnterZone2_3rd, Bezier, 5, Z1toZ2_3,1.7f, -90.f , 0, LockPoint_Brake);
    BezierParam_Init(Trajhandler + Zone1EnterZone2_3rd, Zone1EnterZone2_3rd, SinF, Spin,
                    1.6f, 0.1f, 0.05f, 0.45f, 0.1f, 0.1f, 60, 20);
    //Trajectory_OffPointSet(Trajhandler + Zone1EnterZone2_3rd,200,0);

    vector2d Z1toZ2_2[6] = {    {460,-400},
                                {820,180},
                                {1450,900},
                                {1780,1550},
                                {1940,1640},
                                {2130,1630}};
    TrajParam_SetPoints(Zone1EnterZone2_2nd, Bezier, 5, Z1toZ2_2, 1.8f, -90.f, 0, LockPoint_Brake);
    BezierParam_Init(Trajhandler + Zone1EnterZone2_2nd, Zone1EnterZone2_2nd, SinF, Spin,
                     1.7f, 0.1, 0.05f, 0.45f, 0.1, 0.1f, 80, 20);
    //Trajectory_OffPointSet(Trajhandler + Zone1EnterZone2_3rd,200,0);

    vector2d Z1toZ2_1[6] = {    {460,-400},
                                {820,900},
                                {1450,2150},
                                {1780,2700},
                                {1940,2820},
                                {2130,2830}};
    TrajParam_SetPoints(Zone1EnterZone2_1st, Bezier, 5, Z1toZ2_1, 1.9f, -90.f, 0, LockPoint_Brake);
    BezierParam_Init(Trajhandler + Zone1EnterZone2_1st, Zone1EnterZone2_1st, SinF, Spin,
                     1.8f, 0.1f, 0.05f, 0.45f, 0.1, 0.1f, 100, 20);
    //Trajectory_OffPointSet(Trajhandler + Zone1EnterZone2_3rd,200,0);
    vector2d SlopeToEnd[6] = {
        {8450,  3900},   // P0 坡底，切线朝 +x
        {9800,  3950},   // P1 沿 +x 强力拉出，覆盖坡道段
        {11240, 4150},   // P2 到达坡顶 x 附近，仍朝 +x
        {11300, 2830},   // P3 转弯过渡，x 超出后回弯
        {10850, 1600},   // P4 与 P5 同 x，切线严格朝 -y
        {10350, 1080}  
    };
    TrajParam_SetPoints(Zone3_SlopeToEnd, Bezier, 5, SlopeToEnd, 2.f, 0, -90.f, No_Brake);
    BezierParam_Init(Trajhandler + Zone3_SlopeToEnd, Zone3_SlopeToEnd, SinF, Spin,
                    1.9f, 0.09f, 0.03f, 0.43f, 0.52f, 0.18f, 120, 50);
    
		
}
void Trajctory_CtrlPointReverse(TrajParam *trajparam)
{
    for (int i = 0; i < trajparam->rank + 1; i++)
        trajparam->CtrlPoints[i].y *= -1;
    trajparam->startAngle *= -1;
    trajparam->endAngle   *= -1;
	trajparam->Point_offset.y *= -1;
}
void chassis_SlopeClimb(CHASSIS *chassis)
{
    static u8 arrive_plain = 0;
    switch (chassis->slopeState)
    {
    case SlopeBottom:
        // chassis_Soleniod(true);
        chassis->ChassisPosSet.vx = 2.0f;
        chassis->ChassisPosSet.vy = 0;
        chassis->crossBrake = 0;
        if (fabs(chassis->pitch) > 1.5f)
            chassis->slopeState++;
        break;
    case SlopeBottomEdge:
        chassis->ChassisPosSet.vx = 2.0f;
        chassis->ChassisPosSet.vy = 0.f;
        if (fabs(chassis->pitch + 15.f) < 1.5f)
            chassis->slopeState++;
        break;
    case SlopeOn:
        chassis->ChassisPosSet.vx = 1.6f;
        chassis->ChassisPosSet.vy = 0.f;
        if (fabs(chassis->pitch + 15.f) > 2.f)
            chassis->slopeState++;
        break;
    case SlopeTopEdge:
        chassis->ChassisPosSet.vx = 0.8f - (15.f + chassis->pitch) * 0.05f;
        chassis->ChassisPosSet.vy = 0;
        if (fabs(chassis->pitch) < 0.8f)
        {
            if (++arrive_plain >= 15)
            {
                chassis->slopeState++;
                arrive_plain = 0;
                trajMarker = Zone3_AfterSlope;
                // chassis->IsRunningTraj = true;
            }
        }

        break;
    case SlopeTop:
        chassis->ChassisPosSet.vx = 0.4f;
        chassis->ChassisPosSet.vy = 0;
        chassis->slopeover = 0;
        chassis->slopeState = SlopeBottom;
        sendCarVel(0, 0, 0, RUN_NORMAL);
        return;
        break;
    default:
        break;
    }
    if (chassis->slopeState != SlopeTop)
    {
        chassisLockAngle(chassis, 0.f);
        Chassis_carvelSet(chassis);
        sendCarVel((s16)(chassis->ChassisPosSet.vx * 1000.f),
                   (s16)(chassis->ChassisPosSet.vy * 1000.f),
                   (s16)(chassis->ChassisPosSet.w * 100.f), RUN_NORMAL);
    }
}
