/**
 * @file    myostask.c
 * @brief   底盘主任务、串口通信任务、LED任务、超声波任务和轨迹规划任务实现。
 *          各任务在 FreeRTOS 下周期运行，完成底盘控制的核心功能。
 */
#include "myostask.h"
#include <math.h>

// ======================= 全局变量定义 ==============================

uint16_t tick = 0;        // 全局tick，供Climbover.c的up_down_track使用
bool enable = 0;			// 电机使能请求（置1触发 Motor_Enable）
bool disenable = 0;			// 电机失能请求
bool front_update_req = 0;	// 请求更新前腿速度/加速度限幅
bool back_update_req = 0;	// 请求更新后腿速度/加速度限幅
bool front_update_sent = 0; // 前腿限幅更新已发送标记
bool back_update_sent = 0;	// 后腿限幅更新已发送标记
bool move_ON = 0;			// 手动移动前腿触发标志（调试用）
int angle_test = 0;			// 手动测试角度值（调试用）

bool zero_flag = 0;			 // 寻零完成标志
float front_speed_value = 0; // 待设置的前腿速度上限
float front_acc_value = 0;	 // 待设置的前腿加速度上限
float back_speed_value = 0;	 // 待设置的后腿速度上限
float back_acc_value = 0;	 // 待设置的后腿加速度上限
uint16_t solenoid_flag = 0;	 // 气缸控制标志：bit0=辅助轮, bit2=前气缸, bit3=后气缸

// ===================================================================
//  LED 流水灯任务
// ===================================================================
void Led_Task_Func(void *argument)
{
	for (;;)
	{
		LED_Flow();
	}
}

// ===================================================================
//  串口通信任务
//  说明：周期处理调试串口收发数据包，并刷新气缸状态。
// ===================================================================
void UART_Com_Fuc(void *argument)
{
	for (;;)
	{
		Debug_ProcessRxMsg(&Chassis, &DebugRxPack);
		Debug_ProcessTxMsg(&Chassis, &DebugTxPack);

		solenoid_on(1, (uint8_t)solenoid_flag);

		osDelay(10);
	}
}
// ===================================================================
//  底盘主控制任务
//  说明：5ms 周期运行，完成以下工作：
//        1. 电机使能/失能控制
//        2. 调用状态机周期函数（上台阶/下台阶/站立/R1）
//        3. 底盘自检（周期读错误码）
//        4. 轮速解算及控制指令发送
// ===================================================================
void TaskChassis(void *argument)
{
	TickType_t last_wake_time = osKernelGetTickCount();
	static short askerror = 0;
	for (;;)
	{
		if (enable)
		{
			Motor_Enable(enable, &Chassis);
			enable = 0;
			//			    Master_Response(0, MASTER_MECHANISM_BOX_PRE, 0, 0, 0);
		}
		if (disenable)
		{
			Motor_Enable(0, &Chassis);
			disenable = 0;
		}
		if (Chassis.Ascend)
		{
			Climb_Ascend_Step();
		}
		else if (Chassis.Descend)
		{
			Climb_Descend_Step();
		}
		else if (Chassis.StandUp)
		{
			Climb_UpStand();
		}
		else if (Chassis.ClimbUp2R1)
		{
			Climb_UpToR1();
		}
		else if (Chassis.ClimbUp2R1)
		{
			Climb_BackDown();
		}
		if (move_ON)
		{
			Set_F_Leg_Angle(angle_test);
			move_ON = 0;
		}
		if (Chassis.Enable)
		{
			if (askerror++ > 100)
			{
				askerror = 0;
				ZdriveAsk(0, Err);
			}
			Chassis_SelfCheck(&Chassis);
		}
		chassisSpeed2Wheel(&Chassis);
		if (Chassis.Enable)
			setCtrlMsg(&Chassis);
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CLIMB_TASK_PERIOD_MS));
	}
}

// ===================================================================
//  超声波传感器更新工具函数
//  说明：触发测量、等待结果、记录新旧值及变化量。
// ===================================================================
static void Ultrasonic_UpdateSensor(Ultrasonic_t *u, float *height, float *last, float *change)
{
	Ultrasonic_Trigger(u);
	if (Ultrasonic_WaitForMeasure(u, pdMS_TO_TICKS(50)) != pdTRUE)
		return;

	float new_distance = Ultrasonic_GetDistance(u);

	if (height != NULL)
	{
		if (last != NULL && change != NULL)
		{
			*last = *height;
			*change = fabsf(*last - new_distance);
		}
		*height = new_distance;
	}
}

// ===================================================================
//  超声波测量任务
//  说明：周期读取前后超声波传感器，测量底盘离地高度变化。
// ===================================================================
void UltrasonicTask(void *argument)
{
	for (;;)
	{
		Ultrasonic_UpdateSensor(&ultrasonic_E, &ultrasonic_height_F, &u_height_last_F, &u_height_change_F);
		Ultrasonic_UpdateSensor(&ultrasonic_F, &ultrasonic_height_UP_B, &u_height_last_UP_B, &u_height_change_UP_B);

		osDelay(5);
	}
}

// ===================================================================
//  guiji — 腿部与车轮轨迹规划任务
//  说明：5ms 周期运行，执行腿部角度平滑插值和车轮位置轨迹。
//        腿部轨迹由 Leg_SinHalfMove 处理（所有腿目标角度相同），
//        车轮轨迹由 Set_Wheel_Pos 处理（smoothstep 缓冲）。
// ===================================================================
// 轨迹规划任务

void guiji(void *argument)
{
	static uint16_t tick = 0;
	static uint16_t wheel_tick = 0;
	for (;;)
	{

		if (Wheel_Track_Flag)
		{
			bool wheel_done = Set_Wheel_Pos(Wheel_Track_delta, Wheel_Track_totalTick, wheel_tick);
			if (wheel_done)
			{
				wheel_tick = 0;
				Wheel_Track_Flag = 0;
				UP_OK = 1;
			}
			else
			{
				wheel_tick++;
			}
		}
		else
		{
			wheel_tick = 0;
		}
		osDelay(5);
	}
}

void TINYF(void *argument)
{
	for (;;)
	{
		osDelay(5);
	}
}
