/**
 * @file    myostask.h
 * @brief   底盘主任务、通信任务、LED 任务及轨迹规划任务的头文件。
 *          声明任务函数入口和跨模块共享的标志变量。
 */
#ifndef __MYOSTASK_H__
#define __MYOSTASK_H__

#include "includes.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "DebugCtrl.h"
#include "chassisPara.h"
#include "LED.h"
#include "solenoid.h"
#include "ultrasonic.h"
#include "Climbover.h"

// ==================== 任务函数声明 ====================

/**
 * @brief 串口通信任务：接收/发送调试数据包，控制气缸
 * @param argument 未使用
 */
void UART_Com_Fuc(void *argument);

/**
 * @brief 底盘主控制任务：执行状态机和底盘控制
 *        周期调用 Climb_Ascend_Step / Climb_Descend_Step 等
 * @param argument 未使用
 */
void TaskChassis(void *argument);

/**
 * @brief 爬台阶步进任务（目前预留，未实际使用）
 * @param argument 未使用
 */
void Climbsteps(void *argument);

/**
 * @brief LED 流水灯任务
 * @param argument 未使用
 */
void Led_Task_Func(void *argument);

// ==================== 共享变量声明 ====================

// ---- 腿速度/加速度参数更新标志 ----
extern float front_speed_value;     // 前腿待更新的速度上限值
extern float front_acc_value;       // 前腿待更新的加速度上限值
extern float back_speed_value;      // 后腿待更新的速度上限值
extern float back_acc_value;        // 后腿待更新的加速度上限值

extern bool front_update_req;       // 请求更新前腿限幅
extern bool back_update_req;        // 请求更新后腿限幅
extern bool front_update_sent;      // 前腿更新已发送标记
extern bool back_update_sent;       // 后腿更新已发送标记
extern bool enable;                 // 电机使能控制标志
extern 	uint16_t tick;

#endif /* __MYOSTASK_H__ */
