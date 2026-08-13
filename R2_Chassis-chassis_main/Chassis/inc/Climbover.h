/**
 * @file    Climbover.h
 * @brief   底盘爬台阶/下台阶/站立/R1动作的状态机与控制接口。
 *          定义三状态机枚举、参数结构体、全局变量声明和外部函数接口。
 *          与 Climbover.c 配合，通过周期调用驱动底盘越障。
 */
#ifndef __CLIMBOVER_H__
#define __CLIMBOVER_H__
#include "includes.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassisComm.h"
#include "ultrasonic.h"
#include "ZDrive.h"
#include "myostask.h"
#include "Ultrasound_new.h"
#include "Tinyf.h"

// =========================== 硬件映射定义 ===========================
// ZDrive 电机 ID 映射（1-based 索引）
#define FR_LIFT 9  // 前右腿抬升电机
#define FL_LIFT 8  // 前左腿抬升电机
#define BR_LIFT 11 // 后右腿抬升电机
#define BL_LIFT 10 // 后左腿抬升电机

#define L_WHEEL 12 // 左车轮电机
#define R_WHEEL 13 // 右车轮电机

// ======================= 时间与稳定判定宏 ==========================
#define CLIMB_TASK_PERIOD_MS 5                             // 状态机任务周期 (ms)
#define CLIMB_MS_TO_TICK(ms) ((ms) / CLIMB_TASK_PERIOD_MS) // 毫秒→tick 换算

#define STABLE_WAIT_TINY CLIMB_MS_TO_TICK(50)    //  50ms — 快速判稳（传感器/角度到位判断）
#define STABLE_WAIT_SHORT CLIMB_MS_TO_TICK(100)  // 100ms — 短时等待（纯延时过渡）
#define STABLE_WAIT_MEDIUM CLIMB_MS_TO_TICK(200) // 300ms — 适中等待（降底盘收腿等动作）
#define STABLE_WAIT_LONG CLIMB_MS_TO_TICK(1000)  // 1000ms — 长时间等待（站立等大行程动作）

// ========================= 枚举类型定义 ============================

/**
 * 上台阶状态机枚举
 * 流程: Waiting → Leg_All_Down(抬腿) → Find_Position(气缸定位)
 *       → Front_Leg_Up(收前腿前进) → Go_Forward(前进过渡) → Back_Leg_Up(收后腿完成)
 */
typedef enum
{
    Ascend_Waiting_For_Init, // 等待初始化
    Ascend_Leg_All_Down,     // 四腿伸到台阶高度
    Ascend_Find_Position,    // 开前气缸，底盘前进到台阶边缘
    Ascend_Front_Leg_Up,     // 前腿收回，底盘继续前进让后轮上台阶
    Ascend_Go_Forward,       // 持续前进过渡
    Ascend_Back_Leg_Up,      // 后腿收回，上台阶完成
} Ascend_Process_State;

/**
 * 下台阶状态机枚举
 * 流程: Waiting → On_Position(到边缘) → Front_Leg_Down(前腿放下)
 *       → Find_Position(后退定位) → Back_Leg_Down(后腿放下)
 *       → Go_Forward(前进下台) → Leg_All_Up(收腿完成)
 */
typedef enum
{
    Descend_Waiting_For_Init, // 等待初始化
    Descend_On_Position,      // 底盘前进至台阶边缘
    Descend_Front_Leg_Down,   // 前腿放下接触地面
    Descend_Find_Position,    // 底盘后退/调整，为后腿定位
    Descend_Back_Leg_Down,    // 后腿放下
    Descend_Go_Forward,       // 前进下台阶
    Descend_Leg_All_Up,       // 四条腿收回，下台阶完成
} Descend_Process_State;

/** 上 R1 状态机枚举（与上台阶流程结构类似） */
typedef enum
{
    Ascend_R1_Waiting_For_Init, // 等待初始化
    Ascend_R1_Leg_All_Down,     // 四腿抬升到 R1 高度
    Ascend_R1_Find_Position,    // 后轮轨迹前进定位
    Ascend_R1_Front_Leg_Up,     // 前腿收回
    Ascend_R1_Go_Forward,       // 前进过渡
    Ascend_R1_Back_Leg_Up,      // 后腿收回，完成
} Ascend_R1_Process_State;

/** 子阶段枚举：每个状态机步骤内的执行阶段 */
typedef enum
{
    Enter,   // 刚进入该状态，执行一次初始化动作
    Running, // 正在运行中，持续执行/等待条件
} Action_State;

// ======================= 全局状态声明 ==============================

extern Ascend_Process_State ascend_state;   // 上台阶状态机当前状态
extern Descend_Process_State descend_state; // 下台阶状态机当前状态

// ======================= 参数结构体 ================================

/** 爬台阶/站立等动作的调参结构体，所有角度/速度/加速度单位遵循 ZDrive 规范 */
typedef struct
{
    // ===== 抬升角度（角度制） =====
    float ascend_lift_angle;  // 上台阶时四腿统一抬升的目标角度
    float ascend_stand_angle; // 站立/起身时的目标角度
    float ascend_R1_angle;    // 上R1时的目标角度
    float descend_put_angle;  // 下台阶时腿放下的目标角度

    // ===== 车轮/底盘速度 =====
    float wheel_speed;   // 车轮电机转速（rpm或角度制）
    float chassis_speed; // 底盘麦轮运动速度（上/下台阶时底盘整体速度）

    // ===== 腿的速度/加速度上限 =====
    float ascend_leg_vel_limit;  // 上台阶时腿速度上限
    float ascend_leg_acc_limit;  // 上台阶时腿加速度上限
    float descend_leg_vel_limit; // 下台阶时腿速度上限
    float descend_leg_acc_limit; // 下台阶时腿加速度上限
    float retract_leg_vel_limit; // 回收腿时速度上限
    float retract_leg_acc_limit; // 回收腿时加速度上限

    // ===== 轨迹周期（tick数） =====
    int ascend_track_ticks;  // 上台阶轨迹规划总周期数
    int descend_track_ticks; // 下台阶轨迹规划总周期数

    // ===== 底盘速度（m/s） =====
    float ascend_chassis_vel; // 上台阶时底盘前进速度

    // ===== 传感器阈值 =====
    float front_sensor_threshold; // 前气缸高度突变阈值，用于判断前轮是否搭上台阶

    // ===== R1上台阶 =====
    float R1_wheel_delta_angle; // 上R1时两个后轮需要转动的角度
    int R1_wheel_track_ticks;   // 上R1时后轮轨迹规划周期数
} ClimbConfig;

/** 全局调参实例（可在外部或串口调试时修改） */
extern ClimbConfig ClimbCfg;

// ======================= 函数声明 ==================================

// --- 轨迹规划 ---
void Set_Leg_Angle_Track(float target);                             // 启动四条腿轨迹跟踪到目标角度
void Set_Wheel_Track(float delta, uint16_t totalTick);              // 启动车轮轨迹跟踪
bool Set_Wheel_Pos(float delta, uint16_t totalTick, uint16_t tick); // 车轮平滑位置控制

// --- 外部变量 ---
extern volatile int16_t saved_pos_x;            // 保存的 X 坐标（用于位移判断）
extern volatile int16_t saved_pos_y;            // 保存的 Y 坐标
extern volatile uint16_t step;                  // 轨迹函数辅助步进计数
extern volatile uint16_t totalStep;             // 轨迹函数总周期数
extern volatile bool Track_flag;                // 四条腿轨迹执行标志
extern volatile bool Wheel_Track_Flag;          // 车轮轨迹执行标志
extern volatile float Wheel_Track_delta;        // 车轮轨迹总偏移量
extern volatile uint16_t Wheel_Track_totalTick; // 车轮轨迹总周期
extern volatile float front_leg_track_target;   // 前腿轨迹规划目标角度
extern volatile float back_leg_track_target;    // 后腿轨迹规划目标角度
extern volatile bool UP_OK;                     // 上台阶前腿轨迹/条件完成标志
extern volatile uint8_t box_get;                // 上下台阶是否取箱子
extern volatile bool DEBUG_CHANGE;              // 0=自动运行, 1=手动单步调试
void Leg_SinHalfMove_ResetAll(void);

// --- 主状态机周期调用函数 ---
void Climb_Ascend_Step(void);  // 上台阶状态机主循环
void Climb_Descend_Step(void); // 下台阶状态机主循环
void Climb_UpStand(void);      // 站立动作主循环
void Climb_UpToR1(void);       // 上R1状态机主循环
void Climb_BackDown(void);     // 调试用回归

void Set_F_Leg_Angle(float target); // 设置前两条腿目标角度

// --- 速度/加速度限幅 ---
void Set_Leg_SPEED(float value);       // 设置四条腿速度上限
void Set_Leg_ACC(float value);         // 设置四条腿加速度上限
void Set_Front_Leg_SPEED(float value); // 设置前两条腿速度上限
void Set_Front_Leg_ACC(float value);   // 设置前两条腿加速度上限
void Set_Back_Leg_SPEED(float value);  // 设置后两条腿速度上限
void Set_Back_Leg_ACC(float value);    // 设置后两条腿加速度上限

// --- 其他工具函数 ---
bool Leg_FindZero(uint8_t id);                                                                            // 单腿寻零
bool Leg_SinHalfMove(float str_angle, float target, uint16_t totalTick, uint16_t tick, float *out_angle); // 腿部平滑轨迹
void up_down_track(void);                                                                                 // 抬升/下降轨迹 主循环
// --- 状态切换 ---
void Climb_Ascend_SwitchToNext(void);    // 上台阶切换到下一步
void Climb_Descend_SwitchToNext(void);   // 下台阶切换到下一步
void Climb_Ascend_R1_SwitchToNext(void); // 上R1切换到下一步
void Climb_Ascend_DebugNext(void);       // 上台阶手动触发下一步
void Climb_Descend_DebugNext(void);      // 下台阶手动触发下一步

// --- 高度配置 ---
typedef enum
{
    ClimbHeight_200mm = 200, // 200mm 台阶参数
    ClimbHeight_400mm = 400, // 400mm 台阶参数
    ClimbHeight_Zero = 0,    // 零高度（调试/站立）
} ClimbHeight;

extern ClimbHeight volatile climb_height; // 当前选择的高度
void Climb_SetHeight(ClimbHeight h);      // 根据高度更新参数

// extern Different_position box_position; // 箱子当前位置
extern uint16_t control_solenoid; // 夹爪气缸开关控制

extern bool up_200mm_get_box;
#endif /* __CLIMBOVER_H__ */
