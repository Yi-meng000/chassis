#ifndef __VESCMOTOR_H__
#define __VESCMOTOR_H__

#include "mathFunc.h"
#include "FD_Canqueue.h"
#include "string.h"

#define USE_VESCNUM 4

enum vescMode
{
    VESC_RPM = 0,
    VESC_POSITION,
    VESC_CURRENT,
    VESC_DUTY,
    VESC_HANDBRAKE
};

enum canPacketID // 大概只用前四个吧 一般也只常用速度模式
{
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,		  // 1
	CAN_PACKET_SET_CURRENT_BRAKE, // 2
	CAN_PACKET_SET_RPM,			  // 3
	CAN_PACKET_SET_POS,			  // 4
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5
};
typedef struct
{
    float current; 
    float speed;
    float angle;
    float duty;
    volatile short handbrakeCurrent;
    float angleABS;
}VescValue;

typedef struct VescStatus
{
	bool timeoutflag;
	bool stuckflag;
} VescStatus;

typedef struct 
{
	bool timeoutCheck;
	bool StuckCheck;
	bool releaseWhenStuck;
	uint8_t CurrentLimit;
}VecsLimit;

typedef struct
{
	uint32_t lastRxTime;
	uint32_t TimeoutTick;
	uint32_t stuckCount;

	volatile u16 angleNow;
	volatile u16 anglePre;
	volatile s16 distance;
	volatile int32_t position;
}VescArgum;

typedef struct
{
	volatile bool Enable;
	volatile bool Begin;
	//	volatile bool Brake;
	uint8_t mode;
	uint8_t PolePairs;
	VescValue valSet,valNow;
	VecsLimit limit;
	VescStatus statusflag;
	VescArgum argum;
}VescMOTOR;

extern VescMOTOR Vescmotor[USE_VESCNUM];

void VescReceiveData_CAN2(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);
void VescInit(void);
void VescFunc(void);
void VescPosition_Mode(uint8_t controlID,float position);
void VescRPM_Mode(uint8_t controlID,float speed);
void VescCurrent_Mode(uint8_t controlID,float current);
void VescDuty_Mode(uint8_t controlID,float duty);
void VescBrake_Mode(uint8_t controlID,float handbrake);
void VescStuck_Check(uint8_t id);
void VescTimeOut(uint8_t id);



#endif /* __VESCMOTOR_H__ */



