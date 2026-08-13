#ifndef __CHASSISCOMM_H__
#define __CHASSISCOMM_H__

#include "chassisPara.h"
#include "ZDrive.h"
#include "VescMotor.h"
#include "DJmotor.h"
#include "wheelTrain.h"
#include "Climbover.h"
#include "LED.h"
#define MASTER_CHASSIS_ENABLE 0x01020201 //使能 失能
#define MASTER_CHASSIS_SET_DRIVE_SPEED 0x01020202 // 设置驱动轮速度
#define MASTER_CHASSIS_SET_CARVEL 0x01020203 // 直接设置车身坐标系下速度
#define MASTER_CHASSIS_ASK_DRIVE_SPEED 0x01020212 // 问询驱动轮速度
#define MASTER_CHASSIS_ASK_DRIVE_CURRENT 0x01020213 // 问询驱动轮电流
#define MASTER_CHASSIS_ASK_STEER_ANGLE 0x01020214 // 问询转向轮角度
#define MASTER_CHASSIS_ASK_STEER_CURRENT 0x01020215 // 问询转向轮电流
#define MASTER_CHASSIS_ASCEND 0x01020204 // 抬升
#define MASTER_CHASSIS_DESCEND 0x01020205 // 下降
#define MASTER_CHASSIS_CLIMBW 0x01020206 // 纠偏角速度
#define MASTER_CHASSIS_BETHEONE 0x01020207 // 上R1
#define MASTER_CHASSIS_PUSHUP 0x01020208 // 在R1上称帝（
#define MASTER_CHASSIS_BACKDOWN 0x01020209 // 从R1上回来降下来
#define MASTER_CHASSIS_DEBUG_NEXT 0x0102020A // 单步调试：触发 DEBUG_CHANGE 下的下一阶段

#define MASTER_CHASSIS_ERROR 0x010202EE  // 错误警报
#define MASTER_CHASSIS_RESET 0x010202FF // 复位
#define MASTER_BROADCAST_CHECK 0x01020200 //广播自检

#define MASTER_MECHANISM_BOX_UP_GET 0x03020001 // 发送给机构的取上台阶块通讯
#define MASTER_MECHANISM_BOX_DOWN_GET 0x03020002 // 发送给机构的取下台阶块通讯
#define MASTER_MECHANISM_BOX_200_GET 0x03020003 // 发送给机构的取400台阶块通讯
#define MASTER_MECHANISM_BOX_PRE 0x03020004 // 发送给机构的预取块通讯
#define MASTER_MECHANISM_BOX_DWON_200_GET 0x03020005 // 发送给机构的取特殊-200通讯
#define MASTER_MECHANISM_COUNT 0x03020006 // 发送给机构计数

#define SLAVE_CHASSIS_ENABLE 0x02020101 //使能 失能
#define SLAVE_CHASSIS_SET_DRIVE_SPEED 0x02020102 // 设置驱动轮速度
#define SLAVE_CHASSIS_SET_CARVEL 0x02020103 // 直接设置车身坐标系下速度
#define SLAVE_CHASSIS_ASK_DRIVE_SPEED 0x02020112 // 问询驱动轮速度
#define SLAVE_CHASSIS_ASK_DRIVE_CURRENT 0x02020113 // 问询驱动轮电流
#define SLAVE_CHASSIS_ASK_STEER_ANGLE 0x02020114 // 问询转向轮角度
#define SLAVE_CHASSIS_ASK_STEER_CURRENT 0x02020115 // 问询转向轮电流
#define SLAVE_CHASSIS_ASCEND 0x02020104
#define SLAVE_CHASSIS_DESCEND 0x02020105
#define SLAVE_CHASSIS_CLIMBW 0x02020106 // 纠偏角速度
#define SLAVE_CHASSIS_BETHEONE 0x02020107 // 上R1
#define SLAVE_CHASSIS_PUSHUP 0x02020108 // 在R1上称帝（
#define SLAVE_CHASSIS_BACKDOWN 0x02020109 // 从R1返回
#define SLAVE_CHASSIS_DEBUG_NEXT 0x0202010A // 单步调试：触发 DEBUG_CHANGE 下的下一阶段


#define SLAVE_CHASSIS_ERROR 0x020201EE  // 错误警报
#define SLAVE_CHASSIS_RESET 0x020201FF // 复位
#define SLAVE_CHASSIS_SELFCHECK 0x02020100 //广播自检


#define SLAVE_CHASSIS_ERROR 0x020201EE  // 错误警报
#define SLAVE_CHASSIS_RESET 0x020201FF // 复位
#define SLAVE_CHASSIS_SELFCHECK 0x02020100 //广播自检

#define CHASSIS_DRIVE_RATIO 3.f


void setCtrlMsg(CHASSIS *chassis);
void Master_Response(uint8_t DLC,uint32_t ID,uint8_t Data0,uint8_t Data1,uint8_t Data2);
void Master_CarVel(uint8_t *Rx_data,CHASSIS *chassis);
void Motor_Enable(bool enable,CHASSIS *chassis);
void Drive_RPM_Answer(CHASSIS *chassis);
void Drive_Current_Answer(CHASSIS *chassis);
void Chassis_SelfCheck(CHASSIS *chassis);
void Rotate_Position_Answer(CHASSIS *chassis);
void Rotate_Current_Answer(CHASSIS *chassis);

void ChassisFunc(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);
void setCtrlMsg(CHASSIS *chassis);
void CarVelSet(float vx,float vy,float w,CHASSIS *chassis);

#endif /* __CHASSISCOMM_H__ */

