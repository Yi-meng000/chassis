
#ifndef _SENSORCOM_H_
#define _SENSORCOM_H_

#ifdef __cplusplus
extern "c"
{
#endif

#include "chassisComm.h"
#include "main.h"
#include "sensorparam.h"
#include "MasterComm.h"
#include "FD_Canqueue.h"

    void SENSOR_RecvHandler(FDCAN_RxHeaderTypeDef rxmsg, uint8_t rxdata[]);

    void SENSOR_SetPos(uint8_t ctrlword, s16 x, s16 y, s16 degangle);
    void SENSOR_AskDTData(uint8_t ctrlword, uint8_t times);

#ifdef __cplusplus
}
#endif
#endif
