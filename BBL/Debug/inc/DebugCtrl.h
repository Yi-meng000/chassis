#ifndef __DEBUGCTRL_H__
#define __DEBUGCTRL_H__

#include "MasterComm.h"
#include "usart.h"
#include "includes.h"
#include "wheelTrain.h"
#include "chassisRun.h"
#include "chassisPid.h"
#include "SensorCom.h"
#include "Cameracom.h"
#include "ActcatorCom.h"
#include "RadarMsg.h"
#include "cmsis_os.h"
#include "MapRoute_com.h"
#include "Sekiro.h"
// 数据包配置
#define UNITYCTRL 0 // 使用UNITY调试器

// 数据包配置
#define DEBUG_RX_BOOL_NUM 33 // 接收 bool 数量 20
#define DEBUG_RX_BYTE_NUM 7  // 接收 byte 数量 3

#define DEBUG_RX_SHORT_NUM 7 // 接收 short 数量 6
#define DEBUG_RX_INT_NUM 0   // 接收 int 数量
#define DEBUG_RX_FLOAT_NUM 6 // 接收 float 数量

#define DEBUG_TX_BYTE_NUM 6  // 发送 byte 数量
#define DEBUG_TX_SHORT_NUM 8 // 发送 short 数量
#define DEBUG_TX_INT_NUM 0   // 发送 int 数量
#define DEBUG_TX_FLOAT_NUM 1 // 发送 float 数量

#define DEBUG_PREFIX 0xA5 // 调试器前缀
#define DEBUG_SUFFIX 0x5A // 调试器后缀

#define DEBUG_DATA_LEN 100 // 调试器数据包长度

// 数据包大小
#define TX_PACK_SIZE                                                           \
    (DEBUG_TX_BYTE_NUM + (DEBUG_TX_SHORT_NUM << 1) + (DEBUG_TX_INT_NUM << 2) + \
     (DEBUG_TX_FLOAT_NUM << 2))
#define RX_PACK_SIZE                                       \
    (((DEBUG_RX_BOOL_NUM + 7) >> 3) + DEBUG_RX_BYTE_NUM +  \
     (DEBUG_RX_SHORT_NUM << 1) + (DEBUG_RX_INT_NUM << 2) + \
     (DEBUG_RX_FLOAT_NUM << 2))

typedef struct _debuggermsg
{
    uint8_t RxData[DEBUG_DATA_LEN];                // 接收数据缓冲区
    ALIGN_32BYTES(uint8_t TxData[DEBUG_DATA_LEN]); // 发送数据缓冲区
    uint8_t Prefix;                                // 前缀
    uint8_t Suffix;                                // 后缀
    uint8_t RxDataSize;                            // 接收数据大小
    uint8_t TxDataSize;                            // 发送数据大小
    bool GetPrefix;                                // 是否获取到后缀
    bool GetSuffix;
} DebugMsgTypedef;

// 接收数据包结构
typedef struct _debugrxmsgpack
{
#if DEBUG_RX_BOOL_NUM > 0
    uint8_t Bools[DEBUG_RX_BOOL_NUM]; // 接收布尔值数组
#endif
#if DEBUG_RX_BYTE_NUM > 0
    char Bytes[DEBUG_RX_BYTE_NUM]; // 接收字节数组
#endif
#if DEBUG_RX_SHORT_NUM > 0
    short Shorts[DEBUG_RX_SHORT_NUM]; // 接收 short 数组
#endif
#if DEBUG_RX_INT_NUM > 0
    int Ints[DEBUG_RX_INT_NUM]; // 接收 int 数组
#endif
#if DEBUG_RX_FLOAT_NUM > 0
    float Floats[DEBUG_RX_FLOAT_NUM]; // 接收 float 数组
#endif
    char Space; // 保留空间
} DebugRxMsgPackTypedef;

// 发送数据包结构
typedef struct _debugtxmsgpack
{
#if DEBUG_TX_BYTE_NUM > 0
    char Bytes[DEBUG_TX_BYTE_NUM]; // 发送字节数组
#endif
#if DEBUG_TX_SHORT_NUM > 0
    short Shorts[DEBUG_TX_SHORT_NUM]; // 发送 short 数组
#endif
#if DEBUG_TX_INT_NUM > 0
    int Ints[DEBUG_TX_INT_NUM]; // 发送 int 数组
#endif
#if DEBUG_TX_FLOAT_NUM > 0
    float Floats[DEBUG_TX_FLOAT_NUM]; // 发送 float 数组
#endif
    char Space; // 保留空间
} DebugTxMsgPackTypedef;

typedef struct _roscompack
{
    uint8_t Prefix; // 前缀
    uint8_t enable;
    uint8_t protect;
    float velx;
    float vely;
    float velw;
    uint8_t Suffix; // 后缀
} RosComPackTypedef;

extern DebugMsgTypedef DebugMsg;          // 调试器消息
extern DebugRxMsgPackTypedef DebugRxPack; // 接收数据包
extern DebugTxMsgPackTypedef DebugTxPack; // 发送数据包
extern RosComPackTypedef RosComPack;     // ROS通信数据包


void Debug_Receive(DebugMsgTypedef *debugMsg, uint8_t data);
bool Debug_SendMsg(DebugMsgTypedef *debugMsg,
                   DebugTxMsgPackTypedef *txPack);
bool Debug_ProcessRxMsg(CHASSIS *chassis, DebugRxMsgPackTypedef *rxPack);
void Debug_ProcessTxMsg(CHASSIS *chassis, DebugTxMsgPackTypedef *txPack);
bool Debug_NormalizeRxMsg(DebugMsgTypedef *debugMsg, DebugRxMsgPackTypedef *rxPack);
void ROS2STM_Comtest(RosComPackTypedef *rosPack, uint8_t data[]);
void ROS_Ctrlchassis(CHASSIS *chassis, RosComPackTypedef *rosPack);

#endif /* __DEBUGCTRL_H__ */
