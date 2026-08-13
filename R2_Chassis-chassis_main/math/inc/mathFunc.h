#pragma once 

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#define ABS(x) ((x>0) ? x : -x)
#define GetSign(x) ((x>0) ? 1 : -1)
#define PeakLimit(a,b) if(ABS(a) > ABS(b)) a = GetSign(a) * b
#define Pi 3.1415926f
typedef short s16;
typedef unsigned short u16;

#define EncodeS32Data(f, buff) \
    {                          \
        *(int32_t *)buff = *f; \
    }
#define DecodeS32Data(f, buff) \
    {                          \
        *f = *(int32_t *)buff; \
    }
#define EncodeS16Data(f, buff) \
    {                          \
        *(s16 *)buff = *f;     \
    }
#define DecodeS16Data(f, buff) \
    {                          \
        *f = *(s16 *)buff;     \
    }
#define EncodeU16Data(f, buff) \
    {                          \
        *(u16 *)buff = *f;     \
    }
#define DecodeU16Data(f, buff) \
    {                          \
        *f = *(u16 *)buff;     \
    }


void ChangeDataByte(uint8_t *p1, uint8_t *p2);
float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index);
float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index);
int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index);
int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index);
void buffer_append_int16(uint8_t *buffer, int16_t source, int32_t *index);
double cvtFloat2Double(float n1, float n2);

float uint2float(int x_int, float x_min, float x_max, int bits);
u16 float2uint(float x, float x_min, float x_max, uint8_t bits);
float Lerp(float start,float end,float t);
float N2DEG(float N);
float DEG2RAD(float angle);
float RAD2DEG(float angle);
void Rotate(float* x, float* y, float x0, float y0, float a);
s16 MSG_Byte2Int16(uint8_t *buff, uint8_t i);
int MSG_Byte2Int32(uint8_t *buff, uint8_t i);

void MSG_Int162Byte(s16 data, uint8_t *buff, uint8_t i);
void MSG_Int322Byte(int data, uint8_t *buff, uint8_t i);
