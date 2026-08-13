#ifndef __UART_COMM_H__
#define __UART_COMM_H__

#include"main.h"
#include"stdbool.h"
#include"usart.h"

# define HEAD_CHAR1 0xff
# define HEAD_CHAR2 0xfe
# define TAIL_CHAR1 0x0a
# define TAIL_CHAR2 0x0d
# define UART_DATA_SIZE 75

#define YES_HEAD1 0x59
#define YES_HEAD2 0x53  
#define YES_LENGTH 95
/**
 * @brief 串口结构体
 * head[2]：储存包头
 * tail[2]：储存包尾
 * write：标记写入端在数组中的位置
 * get_head：布尔型变量，判断是否收到了包头
 * tx_data[50]、rx_data[50]：储存接收与发送的数据 、id 以及校验位
 */
typedef struct uart_comm 
{
    unsigned char head[2];
    unsigned char tail[2];
    int write;             
    bool get_head;
    unsigned char tx_data[UART_DATA_SIZE];
    unsigned char rx_data[UART_DATA_SIZE];
    UART_HandleTypeDef *huart;
}uart_comm;
typedef struct Yes_comm 
{
    unsigned char head[2];
    int write;             
    bool get_head;
    bool Rx_flag;
    unsigned char rx_data[YES_LENGTH + 5];
    UART_HandleTypeDef *huart;
}Yes_comm;
//typedef struct uart_queue
//{
//    uint8_t qhead,qtail;
//    uart_comm uc[10];
//}Uart_queue;
//extern Uart_queue uqueue; // 串口的队列 
void uart_comm_init(uart_comm *ucom,UART_HandleTypeDef *huart);
void uart_comm_receive(uart_comm *ucom,unsigned char c);
bool uart_comm_detection(uart_comm *ucom);     //校验函数
void uart_comm_transmit(uart_comm *ucom,char id,char *data,int size);
// void uart_comm_deal(uart_comm *ucom);     
void Uchar2Struct(uart_comm *ucom);
void Struct2Uchar(uart_comm *ucom,uint8_t id);
extern uart_comm uc;
extern Yes_comm YesenseUc;

void YesenseUc_Init(Yes_comm *ycom,UART_HandleTypeDef *huart);
void YesenseUc_receive(Yes_comm *ycom,unsigned char c);
#endif /* __UART_COMM_H__ */
