#ifndef __IRQHANDLER_H__
#define __IRQHANDLER_H__

#include "fdcan.h"
#include "usart.h"
#include "tim.h"
#include "FD_Canqueue.h"
#include "DebugCtrl.h"
#include "includes.h"
#include "ZDrive.h"
#include "VescMotor.h"
#include "DJmotor.h"
#include "my_Unitree.h"
#include "RobotCom.h"
#include "waveform.h"
#define RXBufferNum 64
#define QRCODE_RXLEN 15
#define ROS_PACK_LEN 16
extern __RAM_D1_ ALIGN_32B uint8_t rx_temp1[RXBufferNum];
extern __RAM_D3_ ALIGN_32B uint8_t rx_temp2;
extern __RAM_D1_ ALIGN_32B uint8_t rx_temp3;
extern __RAM_D2_ ALIGN_32B uint8_t rx_temp4;
extern __RAM_D3_ ALIGN_32B uint8_t rx_temp6;
extern __RAM_D2_ ALIGN_32B uint8_t rx_temp9[ROS_PACK_LEN]; 
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif /* __IRQHANDLER_H__ */
