#ifndef TINYF_H
#define TINYF_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// 数据有效性范围配置
#define DISTANCE_MIN    0
#define DISTANCE_MAX    2000    // 根据传感器实际量程调整
#define CONFIDENCE_MAX  100

// 支持的串口数量（根据实际使用的串口号修改）
#define TINYF_PORT_NUM  4       // 最多支持4个串口

// 串口枚举（用户可根据实际使用的串口号增删）
typedef enum {
    TINYF_SOL = 0,			//	气缸
    TINYF_BACK,		//	后
    TINYF_FRONT,		//	前
    TINYF_MID,		//	中
} TinyF_Port_t;

// 每个串口的独立状态结构体
typedef struct {
    volatile uint16_t distance_value;   // 解析得到的距离值
    volatile uint8_t  confidence_value; // 解析得到的置信度
    volatile uint8_t  data_ready;       // 接收完成标志
    
    // 内部状态变量（不需要用户关注）
    uint8_t recv_buf[16];               // 接收缓冲区
    uint8_t index;                      // 当前缓冲区位置
    uint8_t parsing;                    // 解析状态
    uint8_t comma_pos;                  // 逗号位置
} TinyF_Context_t;

// 全局状态数组（每个串口一份）
extern TinyF_Context_t TinyF_CTX[TINYF_PORT_NUM];

// 函数声明
void MINIF_Processing_Data(TinyF_Port_t port, uint8_t RXdata);

#endif
