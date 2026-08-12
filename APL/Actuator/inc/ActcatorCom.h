#ifndef __ACTCATORCOM_H__
#define __ACTCATORCOM_H__

#include "MasterComm.h"
#include "FD_Canqueue.h"

typedef enum _ActuatorState
{
    ACT_DISABLE,
    ACT_READY,
    ACT_BUSY,
    ACT_ERROR,
} ActuatorState;

typedef struct _ActuatorParam
{
    bool enable;
    uint8_t warhead_docked;
    uint8_t warhead_num[3];
    s16 KFS_dis;
    uint8_t KFS_load;
    // bool left_occupied;
    // bool right_occupied;
    ActuatorState arm;
    ActuatorState grab;
} ActuatorParam;
extern ActuatorParam Actparam;
void Actuator_Enable(ActuatorParam *act, uint8_t enable);
void Actuator_ReceiveHandler(ActuatorParam *act, uint8_t rxdata[], FDCAN_RxHeaderTypeDef rxheader);
void Actuator_Reset(ActuatorParam *act);

void Actuator_KFSCatch(ActuatorParam *act, uint8_t mode);
void Actuator_KFSPlace(ActuatorParam *act, uint8_t cell);
// void Actuator_KFSLoad(ActuatorParam *act);
void Actuator_KFSReady(void);
void Actuator_KFSGrabR1Block(ActuatorParam *act);
void Actuator_WarheadCatch(ActuatorParam *act);
// void Actuator_WarheadMove(ActuatorParam *act,bool side);
void Actuator_WarheadReady(ActuatorParam *act, bool side);
void Actuator_WarheadRelease(bool finish);
void Actuator_KFSPrepare(ActuatorParam *act, u8 ready_mode);
void Actuator_DMSetZero(void);
void Actuator_UpClaw(u8 dis);
#endif /* __ACTCATORCOM_H__ */
