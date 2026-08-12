#ifndef __SEKIRO_H__
#define __SEKIRO_H__

#include "tim.h"
#include "includes.h"
#include "relocation.h"
#include "chassisPid.h"
#include "chassisRun.h"
#include "myotask.h"
#include "IRQhandler.h"
#include "Cameracom.h"
#include "MapRoute_com.h"
#include "ActcatorCom.h"
#include "waveform.h"
#include "RobotCom.h"
#define RED 0
#define BLUE 1
#define MEIHUA_FIELD_SIZE 1200

#define CAM_LOCATE 1


#define Arm2Carcenter 205  // mm
#define Craw2Carcenter 168 // mm

#define KFS_EdgeLen 317.5f

#define ZONE1StartX (Sekiro.side == BLUE ? 300 : -300)
#define ZONE1StartY (Sekiro.side == BLUE ? 1400 : -1400)

#define ZONE2StartX 2330
#define ZONE2StartY 250

#define ZONE3_SLOPE_ENTRANCE_X 8400
#define ZONE3_SLOPE_ENTRANCE_Y 3900 * (Sekiro.side == BLUE ? 1 : -1)
// 350 + 100 + 200 * n + Craw2Carcenter
typedef enum MatchZone
{
    ZONE1, // 0 ~ 2000 mm
    ZONE2, // 2000 ~ 9500
    ZONE3, // 9500 ~ 12000
} ZONE;
typedef enum MEIHUA
{
    ZONE2_EXTRANCE,

    ZONE2_MEIHUA1,
    ZONE2_MEIHUA2,
    ZONE2_MEIHUA3,
    ZONE2_MEIHUA4,
    ZONE2_MEIHUA5,
    ZONE2_MEIHUA6,
    ZONE2_MEIHUA7,
    ZONE2_MEIHUA8,
    ZONE2_MEIHUA9,
    ZONE2_MEIHUA10,
    ZONE2_MEIHUA11,
    ZONE2_MEIHUA12,

    ZONE2_EXIT,
    ZONE2_ILLEGAL,
} MEIHUA;
typedef enum KFS
{
    EMPTY,
    R1_KFS,
    R2_KFS,
    FAKE_KFS,
    UNKNOWN_KFS,

} KFS_TYPE;
typedef struct KFS_STATE
{
    vector3d pos;
    float yaw;
    KFS_TYPE type;

} KFS_state;
enum _SignalFromR1
{
    NO_SIGNAL,
    MIDDLEPLACE,
    TOPPLACE,
    PASSR1KFS,
};
enum Direction
{
    No_Dir,
    Forward,
    Backward,
    Left,
    Right,
};
typedef enum
{
    Martial_Mysteries,
    Nine_Palace,
} SkillmatchStage;
typedef enum robot_state
{

    ROBOT_WAITING,
    ROBOT_CHASSISRUN,
    ROBOT_ACCENDING,
    ROBOT_DESCENDING,
    ROBOT_GRABBING_WARHEAD,
    ROBOT_DOCKING_WARHEAD,
    ROBOT_GRABBING_KFS,
    ROBOT_PLACING_KFS,
    ROBOT_R1DOCKING,
    ROBOT_BACKTOEDGE,
    ROBOT_CLIMBSLOPE,
    ROBOT_UPTOR1,
    ROBOT_GRABR1KFS,
    ROBOT_CAMERA_POSADAPT
} Robot_State;
typedef enum task_state
{
    TASK_INIT,
    TASK_PROCESS,
    TASK_FINISH,
    TASK_ERROR,
} Task_State;
typedef struct _field
{
    KFS_TYPE field_KFS; // 12格子中kfs的类型
    uint8_t height;     // 12格子的高度（固定）
    bool R2_KFSDiscard; // 决定是否丢弃R2方块  1为无效应该丢弃的方块
} Field;
typedef struct _Map_Status
{
    uint8_t rear;
    Field field[13];
} Map_Status;

typedef struct _OffLINE_PATH
{
    uint8_t front;
    uint8_t rear;
    uint8_t path[25];
    uint8_t side_pick; // 旁取的数目
    uint8_t KFS_Pos[2];
    uint8_t pick_carPath[2];
} Offline_Path;

typedef struct _MF_lockpoint
{
    Bpoint left_pos;
    Bpoint right_pos;
    Bpoint forwrd_pos;
} MF_Point;

typedef struct _sekiro
{
    bool MatchStart;
    bool SkillMatchStart;
    bool automatic;
    bool half_auto;
    bool Nxt_move;
    bool side; // Red 0 Blue 1 红蓝场
    bool Docked;
    bool ClimbUp2R1;
    bool Slope_Climb;
    bool SearchBefore;
    bool SearchAlready;
    bool LeaveZone1;
    bool getRoutes;
    bool Zone3Restart;
    bool Zone2Restart;
    bool SlopeAfter;
    uint8_t UpClaw;
    uint8_t Movehtal; //横向移动
    uint8_t Leaveway;
    uint8_t DirforKFS;
    uint8_t R1PlaceKFS;   // 来自R1的放置kfs指令
    uint8_t current_zone; // 在那一个区
    uint8_t zone2_field;  // 梅林中当前所在区域
    KFS_state KFS_front;
    Map_Status Map_status;
    MF_Point MF_pos[13];
    Bpoint Zone2Extrance_Pos[3];
    Offline_Path offpath;
    Robot_State state; // 所处动作状态
    Robot_State state_pre;
    Task_State task;
    bool Club_only;
    bool MFcross_only;
    bool Square9_only;
    bool KFS_Ready_Flag;
    Offline_Path retry_path;
} SEKIRO;

extern SEKIRO Sekiro;
extern uint8_t SkillMatchType;
extern uint8_t WarheadFetch_Num;

void Program_Init(void);
void Sekiro_Init(SEKIRO *sekiro);
void MatchZone_Judge(CHASSIS *chassis, SEKIRO *sekiro);
bool MatchMap_Set(uint8_t type, SEKIRO *sekiro);
void Camera_GetKFS(CameraRxMsgPackTypedef *rxPack, CHASSIS *chassis, SEKIRO *sekiro);
bool Route_Get(SEKIRO *sekiro);
bool Route_Load(SEKIRO *sekiro, uint8_t *map);
bool KFSGetTraversal(SEKIRO *sekiro, ActuatorParam *act);
void KFSDiscardDicide(SEKIRO *sekiro);
void MFCrossPos_Init(SEKIRO *sekiro);

void Match_Chassis2Warhead(SEKIRO *sekiro, ActuatorParam *act, CHASSIS *chassis);
void Match_GrabWarhead(SEKIRO *sekiro, ActuatorParam *act);
void Match_WarheadDocking(SEKIRO *sekiro, ActuatorParam *act, CHASSIS *chassis);
void Match_Chassis2Zone2Entrance(SEKIRO *sekiro, CHASSIS *chassis);
void Match_ChassisToTheEdge(SEKIRO *sekiro, CHASSIS *chassis);

Task_State Match_PosUpdate(SEKIRO *sekiro, CameraRxMsgPackTypedef *rxPack, CHASSIS *chassis, u8 InControl);
void Match_PostureAdapt(SEKIRO *sekiro, CHASSIS *chassis);
void Match_GrabKFS(SEKIRO *sekiro, CHASSIS *chassis);
void Match_Ascend(SEKIRO *sekiro, CHASSIS *chassis, bool height, uint8_t grab);
void Match_Descend(SEKIRO *sekiro, CHASSIS *chassis, bool height, uint8_t grab);
bool Camera_Relocation(SEKIRO *sekiro, SENSORUSART_MSG *sense_msg, CameraRxMsgPackTypedef *rxPack, CHASSIS *chassis);
void Match_High400mmGrabKFS(SEKIRO *sekiro, CHASSIS *chassis, ActuatorParam *act);
void Match_CameraChassisAdapt(SEKIRO *sekiro, CHASSIS *chassis);
bool Camera_SearchR1(SEKIRO *sekiro,CHASSIS *chassis);

void Match_KFSPutMiddle(SEKIRO *sekiro, CHASSIS *chassis, ActuatorParam *act);
void Match_KFSPutTop(SEKIRO *sekiro, ActuatorParam *act);
void Match_ClimbSlope(SEKIRO *sekiro, CHASSIS *chassis);
void Match_ChassisToZone3Entrance(SEKIRO *sekiro, CHASSIS *chassis);
void Match_ChassisToSquare9(SEKIRO *sekiro, CHASSIS *chassis);
void Match_UptoR1(SEKIRO *sekiro, CHASSIS *chassis);
void Match_GrabR1KFS(SEKIRO *sekiro, ActuatorParam *act);

void Match_ChassisSearchRoutes(SEKIRO *sekiro, CHASSIS *chassis);
void SkillMatch_GrabKFS(SEKIRO *sekiro, ActuatorParam *act, CHASSIS *chassis);
void SkillMatch_ChassisAdapt(SEKIRO *sekiro, CHASSIS *chassis);
#endif /* __SEKIRO_H__ */
