#ifndef __CHASSISCOMM_H__
#define __CHASSISCOMM_H__

#include "MasterComm.h"
#include "chassisPara.h"
#include "RadarMsg.h"
#include "Cameracom.h"
#include "kalman.h"
void ChassisEnable(uint8_t enable);

void sendCarVel(s16 carVx, s16 carVy, s16 carVw, CHASSIS_RUNMODE mode);
void sendDrivingSpeed(s16 _FL, s16 _FR, s16 _BL, s16 _BR);

void sendChassisAskMsg(uint32_t askCode);
void sendChassisReset(void);
void chassis_ReceiveHandler(CHASSIS *chassis, FDCAN_RxHeaderTypeDef rxheader, uint8_t Rxdata[]);
void chassis_Ascend(CHASSIS *chassis, bool height, uint8_t grab);
void chassis_Descend(CHASSIS *chassis, bool height, uint8_t grab);
void chassis_climbSend(CHASSIS *chassis);
void chassis_Upstand(CHASSIS *chassis, uint8_t up_height);
void chassis_Up2R1(CHASSIS *chassis);
// void chassis_Soleniod(bool solenoid_flag);
void chassis_StepbyStep(CHASSIS *chassis, bool step_flag);
void chassis_SensordataHandle(CHASSIS *chassis, SENSOR_RXPACK *rxPack, SENSORUSART_MSG *msg);
void chassis_DownfromR1(CHASSIS *chassis);
void chassis_Movehorizontal(CHASSIS *chassis,bool pos_set,u8 dis);
#endif /* __CHASSISCOMM_H__ */
