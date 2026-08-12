#ifndef __MAPROUTE_COM_H__
#define __MAPROUTE_COM_H__

#include "includes.h"
#include "usart.h"
#include "RadarMsg.h"
// 数据包配置

#define ROUTE_RX_BYTE_NUM 30 // 接收 byte 数量

#define ROUTE_PREFIX1 0x07 // 前缀
#define ROUTE_PREFIX2 0x08 // 前缀
#define ROUTE_SUFFIX1 0x01 // 后缀
#define ROUTE_SUFFIX2 0x03 // 后缀

#define ROUTE_DATA_LEN 50 // 数据包长度

// 状态
typedef enum _ROUTEstate
{
    ROUTE_STATE_RECEIVING_DONE, // 接收完毕
    ROUTE_STATE_WAIT_PREFIX,    // 等待前缀
    ROUTE_STATE_WAIT_SUFFIX,    // 等待后缀
    ROUTE_STATE_RECEIVING_DATA  // 正在接收数据
} RouteStateTypedef;

// 消息结构
typedef struct _ROUTEmsg
{
    u8 RxData[ROUTE_DATA_LEN];    // 接收数据缓冲区
    u8 TxData[ROUTE_DATA_LEN];    // 发送数据缓冲区
    u8 Prefix;                    // 前缀
    u8 Suffix;                    // 后缀
    u8 RxDataSize;                // 接收数据大小
    u8 TxDataSize;                // 发送数据大小
    RouteStateTypedef RouteState; // 当前状态
    bool Transmit;
    bool Receive;
} RouteMsgTypedef;

// 接收数据包结构
typedef struct _Routerxmsgpack
{
#if ROUTE_RX_BOOL_NUM > 0
    u8 Bools[ROUTE_RX_BOOL_NUM]; // 接收布尔值数组
#endif
#if ROUTE_RX_BYTE_NUM > 0
    char Bytes[ROUTE_RX_BYTE_NUM]; // 接收字节数组
#endif
#if ROUTE_RX_SHORT_NUM > 0
    short Shorts[ROUTE_RX_SHORT_NUM]; // 接收 short 数组
#endif
#if ROUTE_RX_INT_NUM > 0
    int Ints[ROUTE_RX_INT_NUM]; // 接收 int 数组
#endif
#if ROUTE_RX_FLOAT_NUM > 0
    float Floats[ROUTE_RX_FLOAT_NUM]; // 接收 float 数组
#endif
    u8 map_id;
    u8 start_point;
    char Space; // 保留空间
} RouteRxMsgPackTypedef;

extern RouteMsgTypedef RouteMsg;
extern RouteRxMsgPackTypedef RouteRxPack;

bool Route_ReceiveByte(RouteMsgTypedef *routeMsg, u8 data);
bool Route_ProcessRxMsg(RouteMsgTypedef *routeMsg, RouteRxMsgPackTypedef *rxPack);
void Route_SendMsg(uint8_t *map, RouteMsgTypedef *routeMsg);
#endif /* __MAPROUTE_COM_H__ */
