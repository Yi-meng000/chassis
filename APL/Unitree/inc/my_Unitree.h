#ifndef _MY_UNITREE_H
#define _MY_UNITREE_H

#include "gom_protocol.h"
#include "usart.h"
#include "includes.h"
#include <stdbool.h>

#define RS_num 1 // 485总线数量
#define GO_num 3 // 每个RS485总线上的GO电机数量，不要超过4个
#define RS485_RXBUF 32
#define RS485_RXLENGTH 16
#define U3_RS485_Pin GPIO_PIN_10
#define U3_RS485_GPIO_Port GPIOD

typedef struct
{
    MotorCmd_t cmd;
    MotorData_t data;
    volatile bool Enable;
    volatile bool SetZero;
    float set_zero_Pos;
} Go_motor;
extern Go_motor my_GO_motor[RS_num][GO_num];

#define U3_RS485_RxMode() (HAL_GPIO_WritePin(U3_RS485_GPIO_Port, U3_RS485_Pin, GPIO_PIN_RESET))
#define U3_RS485_TxMode() (HAL_GPIO_WritePin(U3_RS485_GPIO_Port, U3_RS485_Pin, GPIO_PIN_SET))

extern MotorCmd_t control_cmd;
extern MotorData_t control_data[RS_num];
extern uint8_t GO_rx_buf[RS_num][2][16];

typedef enum
{
    U3_RS_Channel,
    //	U6_RS_Channel,
    //	U7_RS_Channel,
    //	U8_RS_Channel
} U_RS_Channel;

extern HAL_StatusTypeDef tx_res[RS_num];
extern HAL_StatusTypeDef rx_res[RS_num];

extern bool Unitree_All_Enable_flag;
extern bool Unitree_set_zero_all_flag;

void my_Unitree_Init(void);
void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart,
                                   uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);

void my_Unitree_set_zero(int RS_id, int GO_id);
void Unitree_enable(int RS_id, int GO_id);
void Unitree_disable(int RS_id, int GO_id);
static void U3_Unitree_Function(int index);
void USER_USART_GO_RxHandler(UART_HandleTypeDef *huart, uint16_t Size, uint8_t rs485_index);
void Unitree_GO_Func(void);
// void U6_Unitree_Function(uint8_t index);
// void U7_Unitree_Function(uint8_t index);
// void U8_Unitree_Function(uint8_t index);

#endif
