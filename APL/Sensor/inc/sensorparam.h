/*
 * @Author: nongfu && nongfu63062022@outlook.com
 * @Date: 2025-01-06 00:48:33
 * @LastEditors: nongfu && nongfu63062022@outlook.com
 * @LastEditTime: 2025-01-14 09:02:55
 * @FilePath: \R1Master\APL\Sensor\Inc\sensorparam.h
 * @Description:
 *
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved.
 */

#ifndef _SENSORPARAM_H_
#define _SENSORPARAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    typedef struct _dtdata
    {
        s16 X;
        s16 Y;
        bool GetData;
    } DT_DataTypedef;

    typedef struct _dtdata2f
    {
        float X;
        float Y;
        bool GetData;
    } DT_Data2fTypedef;

    extern DT_Data2fTypedef DTData1;
    extern DT_Data2fTypedef DTData2;
#ifdef __cplusplus
}
#endif

#endif
