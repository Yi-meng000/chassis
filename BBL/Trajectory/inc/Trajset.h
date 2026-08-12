#ifndef __TRAJSET_H__
#define __TRAJSET_H__

#include "includes.h"
#include "Trajectory.h"
#include "chassisPid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "LED.h"
#define LCOKPOINT_NUM 10
#define TRAJ_PERIOD 10
typedef enum trajSpeedMode
{
    Brake,
    SquareRoot,
    Line,
    Square,
    Cube,
    CubeBuffer,
    SinF,
    SelfDefine,
} TrajSpeedMode;

typedef enum trajAngleMode
{
    Spin,
    Follow
} TrajAngleMode;

typedef enum trajName
{
    TestTraj,
    Zone3_AfterSlope,
    Zone2_ToSlopeFrom10,
    Zone2_ToSlopeFrom11,
    Zone2_ToSlopeFrom12,
    Zone1_ToWarehead,
    Zone1EnterZone2_1st,
    Zone1EnterZone2_2nd,
    Zone1EnterZone2_3rd,
    Zone3_BeforeSlope,
    Zone3_SlopeToEnd
} TrajName;
typedef enum trajBrakeMode
{
    Cross_Brake,
    LockPoint_Brake,
    No_Brake,
} TrajBrakeMode;
typedef struct _trajectoryparam
{
    u8 trajMode;
    u8 rank;
    Bpoint CtrlPoints[10];
    float uniformSpeed;
    float startangle;
    float endangle;
    uint8_t iflockpointafterTraj;
} TrajCommonParam;
typedef struct _TrajParam
{
    uint8_t ifLockPointAfterTraj;
    uint8_t trajSpeedMode;
    uint8_t trajAngleMode;
    uint8_t trajMode;

    uint8_t rank;
    Bpoint *CtrlPoints;

    float startSpeed;
    float uniformSpeed;
    float endSpeed;

    float startAngle;
    float endAngle;

    float TotalT;
    float speedUpT;
    float speedDownT;
    float speedUniformT;
    float rotateBeginT;
    float rotateNeedT;

    float SpeedUpS;
    float SpeedUniformS;
    float SpeedDownS;

    float length;
    uint32_t start_time;
    Bpoint chasepoint1, chasepoint2;
    THRESHOLD trajthreshold;
    bool TrajRunInit;
    Bpoint Point_offset;
    bool Inaroll;
} TrajParam;

extern TrajCommonParam trajArray[15];
extern TrajParam Trajhandler[15];
float Trajectory_SpeedUp(float tbegin, float tnow, float tup, float vel_start, float vel_end);
float Trajectory_SpeedDown(float tbegin, float tnow, float tdown, float vel_start, float vel_end);
float TRAJSET_SinSpeed(float T, float tnow, float velstart, float velend);
float Trajectory_Rotate(TrajParam *traj_param);
void TrajParam_SetPoints(TrajName index, uint8_t trajmode, uint8_t rank,
                         vector2d point[],
                         float uniformSpeed, float start_angle, float end_angle,
                         uint8_t ifLockPointAfterTraj);

void BezierParam_Init(TrajParam *param, uint8_t index, uint8_t speedmode, uint8_t anglemode,
                      float startspeed, float endspeed, float speedupS, float speeddownS,
                      float rotatebeginT, float rotateneedT, float usetrajthrehold,
                      float trajdonethreshold);
void Trajectory_OffPointSet(TrajParam *param, s16 offset_x, s16 offset_y);
float GetRatio(TrajParam *param, float runtime);
float GetBezierT(TrajParam *param, float runtime);
bool TrajRun(CHASSIS *chassis, TrajParam *trajparam, Vector2fPID *trajpid, PIDType *lockanglepid, Vector2fPID *lockpointpid, LockpointThresholdTypedef *lockpointThreshold);
#endif /* __TRAJSET_H__ */
