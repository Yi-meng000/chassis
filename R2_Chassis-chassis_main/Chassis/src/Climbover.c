#include "Climbover.h"

/**
 * @file    Climbover.c
 * @brief   底盘爬台阶/下台阶/站立/R1 动作的状态机实现。
 *          包含三个状态机（上台阶/下台阶/上R1）的主循环、轨迹规划函数、
 *          气缸控制、稳定判定工具以及调试步进接口。
 *
 * 整体设计思路：
 * - 每个动作流程对应一个枚举状态机（Ascend_Process_State 等）
 * - 状态机在 Climb_Ascend_Step / Climb_Descend_Step 中周期运行
 * - 每个状态内部用 action_state (Enter/Running) 区分"进入初始化"和"持续运行"
 * - 通过 Climb_Done_Stable 实现防抖判稳，避免传感器抖动导致误切
 * - 腿部运动采用 Leg_SinHalfMove 进行平滑 ease-in-out 轨迹插值
 * - 支持自动运行和手动单步调试两种模式（DEBUG_CHANGE 控制）
 */

// ======================= 全局状态变量 ==============================

/** 上台阶状态机当前状态 */
Ascend_Process_State ascend_state = Ascend_Waiting_For_Init;
/** 下台阶状态机当前状态 */
Descend_Process_State descend_state = Descend_Waiting_For_Init;
/** 上R1状态机当前状态 */
Ascend_R1_Process_State ascend_state_R1 = Ascend_R1_Waiting_For_Init;
/** 子阶段状态：Enter=刚进入的初始化阶段, Running=持续运行阶段 */
Action_State action_state = Enter;
/** 当前箱子的位置状态 */
// Different_position box_position = waiting_position;

// ======================= 标志变量 ==================================

volatile bool UP_OK = 0;      // 上台阶时，前腿是否已达到可以收起的条件
bool F_OK = 0;                // 上台阶时，前腿抬起后的完成判断标志
bool B_OK = 0;                // 上台阶时，后轮是否已经搭在台阶上的判断标志
volatile bool Track_flag = 0; // 四条腿是否正在执行轨迹跟踪移动

// ======================= 气缸控制 ==================================

uint16_t FRONT_LEG = 0x02; // 前气缸标志位
uint16_t BACK_LEG = 0x01;  // 后气缸标志位
/** 实际使用：同时开前气缸+辅助轮 (0x04|0x01) */
uint16_t FUNZHU_FRONT = 0x0A;
/** 实际使用：同时开后气缸+辅助轮 (0x08|0x01) */
uint16_t FUNZHU_BACK = 0x09;

uint16_t FUNZHU = 0x08;

volatile bool front_ok = 0; // 前腿速度/加速度已修改成功的标志
volatile bool back_ok = 0;  // 后腿速度/加速度已修改成功的标志

volatile uint8_t box_get = 0;                // 是否在上下台阶过程中取放方块
volatile bool Wheel_Track_Flag = false;      // 车轮轨迹执行标志
volatile float Wheel_Track_delta = 0.f;      // 车轮轨迹总偏移角度
volatile uint16_t Wheel_Track_totalTick = 0; // 车轮轨迹总周期数

// ======================= 内部辅助变量 ==============================

volatile uint16_t step = 0;        // 轨迹函数辅助步进计数
volatile uint16_t totalStep = 600; // 轨迹函数总周期数 (600tick = 600ms @1ms周期)
volatile uint16_t allstep = 600;   // 四条腿共同轨迹函数总周期数 (600tick = 600ms @1ms周期)
volatile int16_t saved_pos_x = 0;  // 保存的 X 坐标（用于位移判断）
volatile int16_t saved_pos_y = 0;  // 保存的 Y 坐标

volatile float front_leg_track_target = 0; // 前腿轨迹规划目标角度
volatile float back_leg_track_target = 0;  // 前腿轨迹规划目标角度

/** 默认调参结构体（400mm 台阶参数） */
ClimbConfig ClimbCfg = {
    .ascend_lift_angle = 2930.f,  // 改这里：结构体初始默认 / 改 Climb_SetHeight 中各高度分支
    .ascend_stand_angle = 3000.f, // 改这里：结构体初始默认 / 改 Climb_SetHeight 中各高度分支
    .ascend_R1_angle = 3180.f,    // ★ 仅改这里：结构体初始默认，Climb_SetHeight 不复写此参数
    .descend_put_angle = 2900.f,  // 改这里：结构体初始默认 / 改 Climb_SetHeight 中各高度分支
    .wheel_speed = 200.f,         // 改这里：结构体初始默认 / 改 Climb_SetHeight 中各高度分支
    .chassis_speed = 0.7f,        // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .ascend_leg_vel_limit = 20,   // 改这里：结构体初始默认 / 改 Climb_SetHeight 中各高度分支
    .ascend_leg_acc_limit = 200,  // 改这里：结构体初始默认 / 改 Climb_SetHeight 中各高度分支
    .descend_leg_vel_limit = 70,  // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .descend_leg_acc_limit = 700, // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .retract_leg_vel_limit = 40,  // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .retract_leg_acc_limit = 400, // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .ascend_track_ticks = 180,    // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .descend_track_ticks = 200,   // 改这里：结构体初始默认 / 改 Climb_SetHeight 中 200mm/400mm分支
    .ascend_chassis_vel = 0.65f,  // ★ 仅改这里：结构体初始默认，Climb_SetHeight 中三个分支均不复写此参数
    .front_sensor_threshold = 60, // ★ 仅改这里：结构体初始默认，Climb_SetHeight 不复写此参数
    .R1_wheel_delta_angle = 1250, // ★ 仅改这里：结构体初始默认，Climb_SetHeight 不复写此参数
    .R1_wheel_track_ticks = 2500, // ★ 仅改这里：结构体初始默认，Climb_SetHeight 不复写此参数
};

/** 当前选择的抬升高度（默认为200mm） */
ClimbHeight volatile climb_height = ClimbHeight_200mm;
/** 调试/自动切换标志：0=自动运行, 1=手动调试步进 */
volatile bool DEBUG_CHANGE = 0;

/**
 * 单步调试时，记录下一阶段状态。
 * 外部命令调用 Climb_Ascend_DebugNext / Climb_Descend_DebugNext 触发执行下一步。
 */
static Ascend_Process_State ascend_next_state = Ascend_Waiting_For_Init;
static Descend_Process_State descend_next_state = Descend_Waiting_For_Init;

// ===================================================================
//  函数：Climb_SetHeight
//  说明：根据选择的台阶高度（200mm/400mm/Zero），
//        更新 ClimbCfg 中的腿部角度、速度/加速度、轨迹周期等参数。
// ===================================================================
void Climb_SetHeight(ClimbHeight h)
{
    // 根据高度设定统一的抬升角度值
    if (h == ClimbHeight_200mm)
    {
        ClimbCfg.ascend_track_ticks = 80;
        ClimbCfg.ascend_stand_angle = 1650;
        ClimbCfg.wheel_speed = 250.f,
        ClimbCfg.descend_track_ticks = 115;
        ClimbCfg.ascend_lift_angle = 1650.f;
        ClimbCfg.descend_put_angle = 1530.f;
        ClimbCfg.ascend_leg_acc_limit = 220;
        ClimbCfg.ascend_leg_vel_limit = 20;
        ClimbCfg.descend_leg_acc_limit = 300;
        ClimbCfg.descend_leg_vel_limit = 30;
        ClimbCfg.retract_leg_acc_limit = 250;
        ClimbCfg.retract_leg_vel_limit = 30;
        ClimbCfg.chassis_speed = 0.82f;
    }
    else if (h == ClimbHeight_400mm)
    {
        ClimbCfg.ascend_track_ticks = 110;
        ClimbCfg.ascend_stand_angle = 3060;
        ClimbCfg.wheel_speed = 200.f,
        ClimbCfg.descend_track_ticks = 180;
        ClimbCfg.ascend_lift_angle = 2970.f;
        ClimbCfg.descend_put_angle = 2900.f;
        ClimbCfg.ascend_leg_acc_limit = 250;
        ClimbCfg.ascend_leg_vel_limit = 30;
        ClimbCfg.descend_leg_acc_limit = 450;
        ClimbCfg.descend_leg_vel_limit = 50;
        ClimbCfg.retract_leg_acc_limit = 550;
        ClimbCfg.retract_leg_vel_limit = 50;
        ClimbCfg.chassis_speed = 0.8f;
    }
    else if (h == ClimbHeight_Zero)
    {
        ClimbCfg.ascend_leg_acc_limit = 100;
        ClimbCfg.ascend_leg_vel_limit = 15;
        totalStep = 250;
        ClimbCfg.ascend_stand_angle = 100;
        ClimbCfg.ascend_lift_angle = 100;
    }
}

// ===================================================================
//  函数：Leg_SinHalfMove
//  说明：使用二次 ease-in-out 曲线对单腿角度进行平滑轨迹插值。
//         str_angle: 起始角度; target: 目标角度; totalTick: 总周期; tick: 当前周期;out_angle 角度输出
//  返回：true=轨迹已完成, false=仍在运行中
// ===================================================================
bool Leg_SinHalfMove(float str_angle, float target, uint16_t totalTick, uint16_t tick, float *out_angle)
{
    float t;
    float s;
    // 防止传入越界id导致数组访问异常
//    if (id >= USE_ZDRIVE_NUM)
//    {
//        return true;
//    }

    // 计算插值因子
    uint16_t current_tick = (tick > totalTick) ? totalTick : tick;
    t = (float)current_tick / (float)totalTick;

    // 使用二次cos曲线进行平滑插值 (ease-in-out)
    if (t < 0.5f)
    {
        s = 2.0f * t * t;
    }
    else
    {
        s = 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
    }

    // 计算目标角度
    *out_angle = str_angle + (target - str_angle) * s;

    // 轨迹完成时返回true
    if (tick >= totalTick)
    {
        return true;
    }
    return false;
}

static float str_angle[USE_ZDRIVE_NUM] = {0};
static bool initialized[USE_ZDRIVE_NUM] = {0};
static inline void Leg_track_init(uint8_t id)
{
    if (initialized[id])
        return;
    str_angle[id] = Zmotor[id].valSetNow.angle;
    initialized[id] = true;
}
static inline void Leg_track_reset(uint8_t id)
{
    initialized[id] = false;
}

void up_down_track(void)
{
    if (Track_flag)
    {
        static float str_angle_front = 0;
        static float str_angle_back = 0;
        static float angle_front = 0;
        static float angle_back = 0;

        // 首次调用时，记录该腿的起始角度
        Leg_track_init(FL_LIFT);
        Leg_track_init(FR_LIFT);
        Leg_track_init(BL_LIFT);
        Leg_track_init(BR_LIFT);

        str_angle_front = (str_angle[FL_LIFT] + str_angle[FR_LIFT]) / 2;
        str_angle_back = (str_angle[BL_LIFT] + str_angle[BR_LIFT]) / 2;

        bool F_done = Leg_SinHalfMove(str_angle_front, front_leg_track_target, totalStep, tick, &angle_front);
        bool B_done = Leg_SinHalfMove(str_angle_back, front_leg_track_target, totalStep, tick, &angle_back);

        if (F_done && B_done)
        {
            // 停止跟随
            tick = 0;
            Track_flag = 0;

            Leg_track_reset(FL_LIFT);
            Leg_track_reset(FR_LIFT);
            Leg_track_reset(BL_LIFT);
            Leg_track_reset(BR_LIFT);
        }
        else
        { // 设置电机目标角度
            Zmotor[FL_LIFT].valSetNow.angle = angle_front;
            Zmotor[FR_LIFT].valSetNow.angle = angle_front;
            Zmotor[BL_LIFT].valSetNow.angle = angle_back;
            Zmotor[BR_LIFT].valSetNow.angle = angle_back;

            tick++;
        }
    }
    else
    {
        tick = 0; // 如果未启用，保持tick为0
    }
}

/** 设置左右车轮电机的运行模式（速度/位置模式） */
static void Set_Wheel_Mode(float mode)
{
    Zmotor[L_WHEEL].mode = mode;
    Zmotor[R_WHEEL].mode = mode;
}

/** 设置左右车轮电机的目标转速 */
static void Set_Wheel_Speed(float speed)
{
    Zmotor[L_WHEEL].valSetNow.speed = speed;
    Zmotor[R_WHEEL].valSetNow.speed = speed;
}

// ===================================================================
//  函数：Set_Wheel_Track
//  说明：启动车轮平滑轨迹跟踪（在 guiji 任务中周期执行 Set_Wheel_Pos）。
//        delta: 总偏移角度; totalTick: 总周期数
// ===================================================================
void Set_Wheel_Track(float delta, uint16_t totalTick)
{
    Wheel_Track_delta = delta;
    Wheel_Track_totalTick = totalTick;
    Wheel_Track_Flag = true;
}

// ===================================================================
//  函数：Set_Wheel_Pos
//  说明：车轮平滑位置控制（smoothstep 曲线）。
//        起点为当前真实角度，终点为起点+delta。
//        适用于电机力矩有限、难以刹停的场景。
//  返回：true=轨迹完成, false=正在运行
// ===================================================================
bool Set_Wheel_Pos(float delta, uint16_t totalTick, uint16_t tick)
{
    static float wheel_start[2] = {0};
    static bool initialized = false;

    if (totalTick == 0)
    {
        Zmotor[L_WHEEL].valSetNow.angle = Zmotor[L_WHEEL].valSetNow.angle + delta;
        Zmotor[R_WHEEL].valSetNow.angle = Zmotor[R_WHEEL].valSetNow.angle + delta;
        initialized = false;
        return true;
    }

    if (tick == 0 || !initialized)
    {
        wheel_start[0] = Zmotor[L_WHEEL].valReal.angle;
        wheel_start[1] = Zmotor[R_WHEEL].valReal.angle;
        initialized = true;
    }

    uint16_t current_tick = (tick > totalTick) ? totalTick : tick;
    float t = (float)current_tick / (float)totalTick;
    float s = t * t * (3.0f - 2.0f * t);

    float target_angle = delta * s;
    Zmotor[L_WHEEL].valSetNow.angle = wheel_start[0] + target_angle;
    Zmotor[R_WHEEL].valSetNow.angle = wheel_start[1] + target_angle;

    if (tick >= totalTick)
    {
        initialized = false;
        return true;
    }
    return false;
}

// ===================================================================
//  腿部角度设置与到位检查
// ===================================================================

/** 设置前两条腿的目标角度（循环发送，防止电机自行修改设定值） */
void Set_F_Leg_Angle(float target)
{
    front_leg_track_target = target;
    totalStep = 5;
    Track_flag = 1;
}

/** 设置后两条腿的目标角度 */
static void Set_B_Leg_Angle(float target)
{
    back_leg_track_target = target;
    totalStep = 10;
    Track_flag = 1;
}

/** 检查后两条腿是否已到达目标角度（容差±30°） */
static bool Set_B_Leg_Stable(float target)
{
    const float tol = 50.0f; // 角度容差，可根据实际系统调整
    return fabsf(Zmotor[BL_LIFT].valReal.angle - target) < tol && fabsf(Zmotor[BR_LIFT].valReal.angle - target) < tol;
}

/** 检查前两条腿是否已到达目标角度（容差±10°） */
bool Set_F_Leg_Stable(float target)
{
    const float tol = 50.0f; // 角度容差，可根据实际系统调整
    return fabsf(Zmotor[FL_LIFT].valReal.angle - target) < tol && fabsf(Zmotor[FR_LIFT].valReal.angle - target) < tol;
}

// ===================================================================
//  腿部速度/加速度限幅设置
//  注：Zdrive API 使用 1-based ID，所以传入 FL_LIFT+1 等。
// ===================================================================

/** 设置四条腿的速度上限 */
void Set_Leg_SPEED(float value)
{
    ZdriveSetPosVelLimit(value, FL_LIFT + 1);
    ZdriveSetPosVelLimit(value, FR_LIFT + 1);
    ZdriveSetPosVelLimit(value, BL_LIFT + 1);
    ZdriveSetPosVelLimit(value, BR_LIFT + 1);
}

/** 设置四条腿的加速度上限 */
void Set_Leg_ACC(float value)
{
    ZdriveSetAccel(value, FL_LIFT + 1);
    ZdriveSetAccel(value, FR_LIFT + 1);
    ZdriveSetAccel(value, BL_LIFT + 1);
    ZdriveSetAccel(value, BR_LIFT + 1);
}

/** 设置前两条腿的速度上限 */
void Set_Front_Leg_SPEED(float value)
{
    ZdriveSetPosVelLimit(value, FL_LIFT + 1);
    ZdriveSetPosVelLimit(value, FR_LIFT + 1);
}

/** 设置前两条腿的加速度上限 */
void Set_Front_Leg_ACC(float value)
{
    ZdriveSetAccel(value, FL_LIFT + 1);
    ZdriveSetAccel(value, FR_LIFT + 1);
}

/** 设置后两条腿的速度上限 */
void Set_Back_Leg_SPEED(float value)
{
    ZdriveSetPosVelLimit(value, BL_LIFT + 1);
    ZdriveSetPosVelLimit(value, BR_LIFT + 1);
}

/** 设置后两条腿的加速度上限 */
void Set_Back_Leg_ACC(float value)
{
    ZdriveSetAccel(value, BL_LIFT + 1);
    ZdriveSetAccel(value, BR_LIFT + 1);
}

// ===================================================================
//  函数：Set_Leg_Angle_Track
//  说明：将四条腿的目标角度设置为同一值，并启动轨迹跟踪标志。
//        实际轨迹在 guiji 任务中由 Leg_SinHalfMove 周期执行。
// ===================================================================
void Set_Leg_Angle_Track(float target)
{
    totalStep = allstep;

    front_leg_track_target = target;
    back_leg_track_target = target;
    Track_flag = 1;
}

// ===================================================================
//  参数确认工具：检查前后腿的限幅参数是否已成功写入电机
// ===================================================================

/** 检查前腿速度/加速度限幅是否已更新到位（容差0.01） */
bool Is_Acc_Speed_Front_Changed(float acc, float speed)
{
    return (fabsf(Zmotor[FL_LIFT].valReal.velLimit - speed) < 0.01f && fabsf(Zmotor[FR_LIFT].valReal.velLimit - speed) < 0.01f && fabsf(Zmotor[FL_LIFT].valReal.accAcu - acc) < 0.01f && fabsf(Zmotor[FR_LIFT].valReal.accAcu - acc) < 0.01f && fabsf(Zmotor[FL_LIFT].valReal.accDec - acc) < 0.01f && fabsf(Zmotor[FR_LIFT].valReal.accDec - acc) < 0.01f);
}

/** 检查后腿速度/加速度限幅是否已更新到位（容差0.01） */
bool Is_Acc_Speed_Back_Changed(float acc, float speed)
{
    return (fabsf(Zmotor[BL_LIFT].valReal.velLimit - speed) < 0.01f && fabsf(Zmotor[BR_LIFT].valReal.velLimit - speed) < 0.01f && fabsf(Zmotor[BL_LIFT].valReal.accAcu - acc) < 0.01f && fabsf(Zmotor[BR_LIFT].valReal.accAcu - acc) < 0.01f && fabsf(Zmotor[BL_LIFT].valReal.accDec - acc) < 0.01f && fabsf(Zmotor[BR_LIFT].valReal.accDec - acc) < 0.01f);
}

// ===================================================================
//  稳定判定计数器与工具函数
// ===================================================================

/** 稳定判定成功计数器：连续满足条件达到 need 次后返回 true */
static uint16_t counter = 0;
/** 稳定判定失败计数器：连续失败达到 need2 次后重置成功计数 */
static uint16_t fail_counter = 0;

/** 重置稳定判定计数器 */
static inline void Climb_Stable_Reset(void)
{
    counter = 0;
    fail_counter = 0;
}

/**
 * Climb_Done_Stable — 防抖稳定的条件判定函数
 * @param condition 待判定的布尔条件
 * @param need      条件连续成立 need 次后才判定为 true（防误判）
 * @param need2     条件不成立时，连续失败 need2 次后重置成功计数（防短暂毛刺）
 * @return true=条件已稳定满足
 *
 * 设计思路：条件成立时累加成功计数，失败时累加失败计数。
 * 成功达到 need 次后返回 true 并自动复位。
 * 失败累计到 need2 次后清零成功计数，防止传感器短暂抖动导致误触发。
 */
bool Climb_Done_Stable(bool condition, uint16_t need, uint16_t need2)
{
    if (condition)
    {
        fail_counter = 0; // 满足条件后才会清空失败计数
        if (++counter >= need)
        {
            counter = 0; // 成功后自动复位
            fail_counter = 0;
            return true;
        }
        return false;
    }
    else
    {
        // 条件不满足：累计失败次数，达到need2次才清零计数器
        if (++fail_counter >= need2)
        {
            counter = 0;
        }
        return false;
    }
}

// ===================================================================
//  状态切换助手
//  功能：统一重置 action_state 和稳定计数器，
//        自动模式下递增到下一步，调试模式下记录下一步后回到等待态。
// ===================================================================

/** 上台阶状态机切换到下一步 */
void Climb_Ascend_SwitchToNext(void)
{
    action_state = Enter;
    Climb_Stable_Reset();
    Climb_Done_Stable(true, 0, 0);
    if (DEBUG_CHANGE)
    {
        // 调试模式：记录下一阶段，并回到等待态，等待人工下一步触发
        if (ascend_state < Ascend_Back_Leg_Up)
            ascend_next_state = (Ascend_Process_State)(ascend_state + 1);
        else
            ascend_next_state = Ascend_Waiting_For_Init;
        ascend_state = Ascend_Waiting_For_Init;
    }
    else
    {
        // 正常模式：进入后续枚举状态；越界保护回到等待态
        if (ascend_state < Ascend_Back_Leg_Up)
            ascend_state = (Ascend_Process_State)(ascend_state + 1);
        else
            ascend_state = Ascend_Waiting_For_Init;
    }
}

/** 下台阶状态机切换到下一步 */
void Climb_Descend_SwitchToNext(void)
{
    action_state = Enter;
    Climb_Stable_Reset();
    Climb_Done_Stable(true, 0, 0);
    if (DEBUG_CHANGE)
    {
        // 调试模式：记录下一阶段，并回到等待态，等待人工下一步触发
        if (descend_state < Descend_Leg_All_Up)
            descend_next_state = (Descend_Process_State)(descend_state + 1);
        else
            descend_next_state = Descend_Waiting_For_Init;
        descend_state = Descend_Waiting_For_Init;
    }
    else
    {
        // 正常模式：进入后续枚举状态；越界保护回到等待态
        if (descend_state < Descend_Leg_All_Up)
            descend_state = (Descend_Process_State)(descend_state + 1);
        else
            descend_state = Descend_Waiting_For_Init;
    }
}

/** 调试模式：上台阶手动触发下一步 */
void Climb_Ascend_DebugNext(void)
{
    if (!DEBUG_CHANGE)
        return;

    if (ascend_state != Ascend_Waiting_For_Init)
        return;

    if (ascend_next_state == Ascend_Waiting_For_Init)
        ascend_state = Ascend_Leg_All_Down;
    else
        ascend_state = ascend_next_state;

    ascend_next_state = Ascend_Waiting_For_Init;
    action_state = Enter;
    Climb_Stable_Reset();
}

/** 调试模式：下台阶手动触发下一步 */
void Climb_Descend_DebugNext(void)
{
    if (!DEBUG_CHANGE)
        return;

    if (descend_state != Descend_Waiting_For_Init)
        return;

    if (descend_next_state == Descend_Waiting_For_Init)
        descend_state = Descend_On_Position;
    else
        descend_state = descend_next_state;

    descend_next_state = Descend_Waiting_For_Init;
    action_state = Enter;
    Climb_Stable_Reset();
}

/** R1 上台阶状态机切换到下一步（目前调试模式统一回到等待态） */
void Climb_Ascend_R1_SwitchToNext(void)
{
    action_state = Enter;
    Climb_Stable_Reset();
    Climb_Done_Stable(true, 0, 0);
    if (DEBUG_CHANGE)
    {
        // 调试模式：统一回到等待态，便于人工步进
        ascend_state_R1 = Ascend_R1_Waiting_For_Init;
    }
    else
    {
        // 正常模式：进入后续枚举状态；越界保护回到等待态
        if (ascend_state_R1 < Ascend_R1_Back_Leg_Up)
            ascend_state_R1 = (Ascend_R1_Process_State)(ascend_state_R1 + 1);
        else
            ascend_state_R1 = Ascend_R1_Waiting_For_Init;
    }
}

// ===================================================================
//  上台阶状态机主循环
//  说明：根据 ascend_state 执行对应阶段动作，周期调用。
//  流程: Waiting → Leg_All_Down(抬腿) → Find_Position(气缸定位前进)
//        → Front_Leg_Up(收前腿前进) → Go_Forward(前进过渡)
//        → Back_Leg_Up(收后腿完成)
// ===================================================================
void Climb_Ascend_Step(void)
{
    switch (ascend_state)
    {
    case Ascend_Waiting_For_Init:
        // 等待初始化进入上台阶流程
        // 调试模式下仅打开车轮电机，使能状态保持等待
        if (DEBUG_CHANGE)
        {
            action_state = Enter;
        }
        else
        {
            // 自动模式：直接进入第一阶段
            Climb_Ascend_SwitchToNext();
        }
        break;
    case Ascend_Leg_All_Down:
        Set_Front_Leg_SPEED(ClimbCfg.ascend_leg_vel_limit); // 上台阶时为了稳定选择较低速度和加速度
        Set_Front_Leg_ACC(ClimbCfg.ascend_leg_acc_limit);
        Set_Back_Leg_SPEED(ClimbCfg.ascend_leg_vel_limit);
        Set_Back_Leg_ACC(ClimbCfg.ascend_leg_acc_limit);
        front_ok = Is_Acc_Speed_Front_Changed(ClimbCfg.ascend_leg_acc_limit, ClimbCfg.ascend_leg_vel_limit);
        back_ok = Is_Acc_Speed_Back_Changed(ClimbCfg.ascend_leg_acc_limit, ClimbCfg.ascend_leg_vel_limit);
        if (action_state == Enter)
        {
//            EventStartA(0);
            Set_Wheel_Mode(Zdrive_Speed);          // 切后轮为速度模式
            allstep = ClimbCfg.ascend_track_ticks; // 抬升腿轨迹修改为上台阶周期
            if (!DEBUG_CHANGE)
            {
                // 自动模式下根据设置高度加载爬升参数
                Climb_SetHeight(climb_height);
            }
            // 现有行为：直接开始轨迹跟踪抬升
            if (front_ok && back_ok)
            { // 确保腿的参数修改成功
                front_ok = 0;
                back_ok = 0;
                action_state = Running;
                Set_Leg_Angle_Track(ClimbCfg.ascend_lift_angle); // 启动轨迹开始抬升
            }
        }
        if (Climb_Done_Stable(Set_F_Leg_Stable(ClimbCfg.ascend_lift_angle) && Track_flag == 0, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20))) // 确认前腿位置来保证气缸不会由于离太近而导致打到台阶
        {
            Climb_Ascend_SwitchToNext();

            Set_Front_Leg_SPEED(ClimbCfg.retract_leg_vel_limit); // 上台阶收腿要快一点，此处四条腿不动能够修改对应参数
            Set_Front_Leg_ACC(ClimbCfg.retract_leg_acc_limit);
            Set_Back_Leg_SPEED(ClimbCfg.retract_leg_vel_limit);
            Set_Back_Leg_ACC(ClimbCfg.retract_leg_acc_limit);
        }
        break;
    case Ascend_Find_Position:
        Set_Front_Leg_SPEED(ClimbCfg.retract_leg_vel_limit); // 上台阶收腿要快一点，此处四条腿不动能够修改对应参数
        Set_Front_Leg_ACC(ClimbCfg.retract_leg_acc_limit);
        Set_Back_Leg_SPEED(ClimbCfg.retract_leg_vel_limit);
        Set_Back_Leg_ACC(ClimbCfg.retract_leg_acc_limit);
        front_ok = Is_Acc_Speed_Front_Changed(ClimbCfg.ascend_leg_acc_limit, ClimbCfg.ascend_leg_vel_limit);
        back_ok = Is_Acc_Speed_Back_Changed(ClimbCfg.ascend_leg_acc_limit, ClimbCfg.ascend_leg_vel_limit);
        if (action_state == Enter)
        {
            solenoid_flag = FUNZHU_FRONT;          // 打开前气缸
            Set_Wheel_Speed(ClimbCfg.wheel_speed); // 设置车轮电机转速
            action_state = Running;
        }
        if (front_ok && back_ok)
        {
            UP_OK = 1;
            front_ok = 0;
            back_ok = 0;
        }
        // 若标志位置一，则进入取块动作
        if (box_get == 1)
        {
            Master_Response(0, MASTER_MECHANISM_BOX_UP_GET, 0, 0, 0);
        }
        else if (box_get == 2)
        {
            Master_Response(0, MASTER_MECHANISM_BOX_PRE, 0, 0, 0);
        }
        // 第二阶段结束：等待前腿收回条件并执行收回动作
        if (Climb_Done_Stable(UP_OK && TinyF_CTX[TINYF_SOL].distance_value > 15 && TinyF_CTX[TINYF_SOL].distance_value < ClimbCfg.front_sensor_threshold, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
        {
            Set_F_Leg_Angle(100);
            Set_Wheel_Speed(0);
            UP_OK = 0;
            Climb_Ascend_SwitchToNext();
        }
        break;
    case Ascend_Front_Leg_Up:
        if (action_state == Enter)
        {
            if (Set_F_Leg_Stable(100))
            {                                                           // 保证前腿收了起来再走，防止前腿撞台阶
                CarVelSet(ClimbCfg.ascend_chassis_vel, 0, 0, &Chassis); // 车体前进
                Set_Wheel_Speed(400);                                   // 设置车轮电机转速
                UP_OK = 1;
                action_state = Running;
            }
        }
        // 后轮落位后，再等待一个稳定区间，防止底盘后仰
        if (Climb_Done_Stable(UP_OK && TinyF_CTX[TINYF_MID].distance_value > 20 && TinyF_CTX[TINYF_MID].distance_value < 55, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20))) // 用中间的光电，保证底盘速度就能够稳定上去且后腿不会撞台阶
        {
            UP_OK = 0;
            Set_Wheel_Speed(0);
            Set_B_Leg_Angle(100); // 收后腿
            Climb_Ascend_SwitchToNext();
        }
        break;
    case Ascend_Go_Forward:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Climb_Done_Stable(1, STABLE_WAIT_SHORT, CLIMB_MS_TO_TICK(20)))
        {
            Climb_Ascend_SwitchToNext();
        }
        break;
    case Ascend_Back_Leg_Up:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Climb_Done_Stable(TinyF_CTX[TINYF_BACK].distance_value > 20 && TinyF_CTX[TINYF_BACK].distance_value < 55, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
        {
            solenoid_flag = false; // 气缸最后收，防止电磁阀版发癫
            CarVelSet(0, 0, 0, &Chassis);
//            EventStopA(0);
            if (Set_B_Leg_Stable(100))
            {
                Chassis.Ascend = 0;
                Master_Response(2, SLAVE_CHASSIS_ASCEND, 'O', 'K', 0); // 回复主控
                Climb_Ascend_SwitchToNext();
            }
        }
        break;
    default:
        break;
    }
}

// ===================================================================
//  下台阶状态机主循环
//  说明：根据 descend_state 执行对应阶段动作，周期调用。
//  流程: Waiting → On_Position(前进到边缘) → Front_Leg_Down(前腿放下)
//        → Find_Position(后退定位后腿) → Back_Leg_Down(后腿放下)
//        → Go_Forward(前进下台) → Leg_All_Up(收腿完成)
// ===================================================================
// 下台阶主循环函数：根据当前 descend_state 执行对应阶段动作
// 整体流程：初始化 -> 前往下降位置 -> 前腿放下 -> 底盘后仰定位 -> 后腿放下 -> 完成收腿
void Climb_Descend_Step(void)
{
    switch (descend_state)
    {
    case Descend_Waiting_For_Init:
        // 等待初始化并打开车轮电机
        action_state = Enter;
        counter = 0;
        if (!DEBUG_CHANGE)
        {
            Climb_SetHeight(climb_height); // 更新当前高度参数
            Climb_Descend_SwitchToNext();  // 直接进入下台阶第一阶段
        }
        break;
    case Descend_On_Position:
        Set_Front_Leg_SPEED(ClimbCfg.descend_leg_vel_limit); // 下台阶时腿要出的快一点，但是不能加速度太大，要不然车身不稳
        Set_Front_Leg_ACC(ClimbCfg.descend_leg_acc_limit);
        Set_Back_Leg_SPEED(ClimbCfg.descend_leg_vel_limit);
        Set_Back_Leg_ACC(ClimbCfg.descend_leg_acc_limit);
        if (action_state == Enter)
        {
            solenoid_flag = FUNZHU;
            Set_Wheel_Mode(Zdrive_Speed);
            allstep = ClimbCfg.descend_track_ticks; // 切换最后降底盘的周期，防止对底盘损伤，比赛的时候或许可以适当调低一点让底盘降的快一点
            CarVelSet(ClimbCfg.chassis_speed, 0, 0, &Chassis);
            action_state = Running;
            if (box_get == 1)
            {
                Master_Response(0, MASTER_MECHANISM_BOX_DOWN_GET, 0, 0, 0);
            }
            else if (box_get == 2)
            {
                Master_Response(0, MASTER_MECHANISM_BOX_DWON_200_GET, 0, 0, 0);
            }
        }
        if (Climb_Done_Stable(TinyF_CTX[TINYF_FRONT].distance_value > 150 && TinyF_CTX[TINYF_FRONT].distance_value < 700, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20))) // 用前光电来判断是否能够伸前腿
        {
            Set_F_Leg_Angle(ClimbCfg.descend_put_angle);
            Climb_Descend_SwitchToNext();
        }
        break;
    case Descend_Front_Leg_Down:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Set_F_Leg_Stable(ClimbCfg.descend_put_angle))
        {
            Climb_Descend_SwitchToNext();
        }
        break;
    case Descend_Find_Position:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (TinyF_CTX[TINYF_MID].distance_value > 150 && TinyF_CTX[TINYF_MID].distance_value < 700)
        {
            solenoid_flag = FUNZHU_BACK;
        }
        if (TinyF_CTX[TINYF_BACK].distance_value > 150 && TinyF_CTX[TINYF_BACK].distance_value < 700)
        {
            CarVelSet(0, 0, 0, &Chassis);
            Set_B_Leg_Angle(ClimbCfg.descend_put_angle);
            Climb_Descend_SwitchToNext();
        }
        break;
    case Descend_Back_Leg_Down:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Set_B_Leg_Stable(ClimbCfg.descend_put_angle)) // 确认后腿到位
        {
            solenoid_flag = false; // 收回气缸
            Climb_Descend_SwitchToNext();
        }
        break;
    case Descend_Go_Forward:
        if (action_state == Enter)
        {
            allstep = ClimbCfg.descend_track_ticks;
            action_state = Running;
        }
        if (Climb_Done_Stable(1, STABLE_WAIT_MEDIUM, CLIMB_MS_TO_TICK(20)))
        {
            Climb_Descend_SwitchToNext();
        }
        break;
    case Descend_Leg_All_Up:
        if (action_state == Enter)
        {
            Set_Leg_Angle_Track(100); // 启动四条腿的轨迹函数来降底盘
            action_state = Running;
        }
        if (Set_B_Leg_Stable(100) && Track_flag == 0)
        {
            Chassis.Descend = 0;
            Master_Response(2, SLAVE_CHASSIS_DESCEND, 'O', 'K', 0);
            Climb_Descend_SwitchToNext();
        }
        break;
    default:
        break;
    }
}

// ===================================================================
//  函数：Climb_UpStand
//  说明：站立/起身动作，采用统一 1.5s 总周期防止震荡。
//        实际应用中只使用 ClimbHeight_Zero 和 ClimbHeight_400mm。
// ===================================================================
// 起身动作函数：用于站立动作，采用统一轨迹周期防止震荡，实际中只会跑0和400
bool up_200mm_get_box = 0;
void Climb_UpStand(void)
{
    // 统一为1.5s周期，防止震荡和对底盘损伤
    allstep = 300;
    // 执行一次轨迹函数，防止任务栈爆掉
    if (action_state == Enter)
    {
        osDelay(10);
        Set_Leg_Angle_Track(ClimbCfg.ascend_stand_angle);
        action_state = Running;
    }
    if (up_200mm_get_box && Set_F_Leg_Stable(ClimbCfg.ascend_stand_angle) && Track_flag == 0)
    {
        up_200mm_get_box = 0;
        Master_Response(0, MASTER_MECHANISM_BOX_200_GET, 0, 0, 0);
    }
    if (Climb_Done_Stable(Set_F_Leg_Stable(ClimbCfg.ascend_stand_angle), STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
    {
        action_state = Enter;
        Chassis.StandUp = 0;
        Master_Response(2, SLAVE_CHASSIS_PUSHUP, 'O', 'K', 0);
    }
}

// ===================================================================
//  函数：Climb_UpToR1
//  说明：上 R1 状态机循环，流程与上台阶类似。
//        使用后轮位置模式轨迹前进，防止力矩不足导致刹停不稳。
//  流程: Waiting → Leg_All_Down(抬腿+后轮切位置) → Find_Position(后轮轨迹)
//        → Front_Leg_Up(收前腿) → Go_Forward(前进) → Back_Leg_Up(收后腿完成)
// ===================================================================
// 上R1流程暂时同上台阶
void Climb_UpToR1(void)
{
    switch (ascend_state_R1)
    {
    case Ascend_R1_Waiting_For_Init:
        if (DEBUG_CHANGE)
        {
            action_state = Enter;
        }
        else
        {
            // 自动模式：直接进入第一阶段
            Climb_Ascend_R1_SwitchToNext();
        }
        break;
    case Ascend_R1_Leg_All_Down:
        if (action_state == Enter)
        {
//            EventStartA(0);

            static float Z_angle_L = 0;
            Z_angle_L = Zmotor[L_WHEEL].valReal.angle;
            static float Z_angle_R = 0;
            Z_angle_R = Zmotor[R_WHEEL].valReal.angle; // 辅助两个后轮走一个轨迹用的，把轮子当前位置赋给setnowangle，防止切到位置模式轮子疯转

            Zmotor[L_WHEEL].valSetNow.angle = Z_angle_L;
            Zmotor[R_WHEEL].valSetNow.angle = Z_angle_R;

            Set_Wheel_Mode(Zdrive_Postion);                // 切为位置模式走一个固定距离
            allstep = 180;                                 // 切腿抬升周期
            Set_Leg_Angle_Track(ClimbCfg.ascend_R1_angle); // 腿抬升
            action_state = Running;
        }
        if (Climb_Done_Stable(Set_F_Leg_Stable(ClimbCfg.ascend_R1_angle) && Track_flag == 0, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20))) // 站定再开始走
        {
            Climb_Ascend_R1_SwitchToNext();
        }
        break;
    case Ascend_R1_Find_Position:
        if (action_state == Enter)
        {
            Set_Wheel_Track(ClimbCfg.R1_wheel_delta_angle, CLIMB_MS_TO_TICK(ClimbCfg.R1_wheel_track_ticks)); // 后轮走一个轨迹，防止因力矩不够导致车子停不住
            action_state = Running;
        }
        if (Climb_Done_Stable(1, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
        {
            Climb_Ascend_R1_SwitchToNext();
        }
        break;
    case Ascend_R1_Front_Leg_Up:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Climb_Done_Stable(1, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
        {
            Climb_Ascend_R1_SwitchToNext();
        }
        break;
    case Ascend_R1_Go_Forward:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Climb_Done_Stable(1, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
        {
            Climb_Ascend_R1_SwitchToNext();
        }
        break;
    case Ascend_R1_Back_Leg_Up:
        if (action_state == Enter)
        {
            action_state = Running;
        }
        if (Climb_Done_Stable(Wheel_Track_Flag == 0, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
        {
            Chassis.ClimbUp2R1 = 0;
            Master_Response(2, SLAVE_CHASSIS_BETHEONE, 'O', 'K', 0);
//            EventStopA(0);
            Climb_Ascend_R1_SwitchToNext();
        }
        break;
    default:
        break;
    }
}

void Climb_BackDown(void)
{
    // 统一为1.5s周期，防止震荡和对底盘损伤
    allstep = 300;
    // 执行一次轨迹函数，防止任务栈爆掉
    if (action_state == Enter)
    {
        osDelay(10);
        Set_Wheel_Track(-ClimbCfg.R1_wheel_delta_angle, CLIMB_MS_TO_TICK(ClimbCfg.R1_wheel_track_ticks)); // 后轮走一个轨迹，防止因力矩不够导致车子停不住
        action_state = Running;
    }
    if (Climb_Done_Stable(Wheel_Track_Flag == 0, STABLE_WAIT_TINY, CLIMB_MS_TO_TICK(20)))
    {
        Set_Leg_Angle_Track(100);
        if (Track_flag == 0)
        {
            action_state = Enter;
            Chassis.ClimbBackDown = 0;
            Master_Response(2, SLAVE_CHASSIS_BACKDOWN, 'O', 'K', 0);
        }
    }
}
