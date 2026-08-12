#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "includes.h"
#include "vector.h"
#define WINDOW_SIZE 5
typedef struct _kalman
{
    float Q; // 预测的噪声最小协方差
    float R; // 观测的噪声最小协方差

    float x_last, x_mid, x_now; // mid为先验
    float p_last, p_mid, p_now; // 误差的协方差矩阵

    float kg; // 卡尔曼系数

    float A; // 状态转移矩阵
    float H; // 转换矩阵本应为矩阵 一维形式
    // x(n) = Ax(n-1) + u(n)
    // z(n) = Hx(n) + v(n)
} Kalman;

#define EMA_USE 1
#define EMA_ALPHA_MIN 0.2f
#define EMA_ALPHA_MAX 0.8f
#define EMA_ALPHA_CHANGETHRESH 15
#define MAD_THRESHOLD 3
#define MAD_CORRECTTHRESH 0.6745f
typedef struct _Filter_State
{
    float buf[WINDOW_SIZE];
    uint8_t head, count;
    bool output_valid;
    float EMA_pre;
    bool EMA_initialize;
} Filter_State;

typedef enum
{
    NO_FILTER,
    AVERAGE,
    MEDIAN,
} Filter_Mode;

typedef struct _Sensor_Filter
{
    Filter_State f_x;
    Filter_State f_y;
    Filter_Mode mode;
    float output;
} Sensor_Filter;

extern Sensor_Filter Radar_Filter;
void Kalman_Init(Kalman *kal, float _Q, float _R);
float Kalman_Filter(Kalman *kal, float input); // input 为观测数据
void Filter_Init(Sensor_Filter *filter);
void Filter_BuffPush(Filter_State *state, float value);
void Filter_BuffLinearize(Filter_State *state, float *out_data);
float MAD_Caculate(float *data, uint8_t len);
float Filter_alpha_Adjust(float change);
float Filter_Average(Filter_State *state, float raw_data);
float Filter_Median(Filter_State *state, float raw_data);
void Filter_Func(Sensor_Filter *filter, vector2d raw_data, vector2d *output);

#endif /* __KALMAN_H__ */
