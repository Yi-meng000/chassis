#pragma once

#include "mathFunc.h"
#include "includes.h"
#include "vector.h"
enum PIDMode
{
    PIDPOS, // 位置式PID
    PIDINC, // 增量式PID
};

typedef struct
{
    float KP;
    float KI;
    float KD;
    volatile float SetVal; // 设置值
    volatile float CurVal; // 当前值
    volatile float err[3]; // 误差
    volatile float output; // 输出量
    uint8_t mode;
} PIDType;

typedef struct _vector2fPID
{
    float kp;
    float ki;
    float kd;
    uint8_t mode;
    volatile vector2d input;
    volatile vector2d target;
    volatile vector2d output;
    volatile vector2d err[3];
} Vector2fPID;

void PID_Init(PIDType *pid, float kp, float ki, float kd, uint8_t mode); // 初始化
float PID_Caculate(PIDType *pid, float Input, float Target);

void vector2fPIDInit(Vector2fPID *pid, float *param, uint8_t mode);
vector2d vector2fPIDOperation(Vector2fPID *pid);
