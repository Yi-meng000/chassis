
#ifndef _RELOCATION_H_
#define _RELOCATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "sensorcom.h"
#include "task.h"
#include "cmsis_os.h"
#define DT_RELOCCATION_OFFSET_THRESHOLD 100000 // DT允许偏差范围z
#define FIELD_XDISTENSE 1200
#define FIELD_YDISTENSE 1200

    void SENSOR_Relocation(uint8_t ctrlword, uint8_t times, uint8_t relocmode, uint8_t zone);

    enum DT_RelocMode
    {
        RelocX,
        RelocY,
        RelocXY
    };

    extern DT_Data2fTypedef ErrData;

#ifdef __cplusplus
}
#endif
#endif
