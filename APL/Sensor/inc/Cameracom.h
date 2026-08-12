#ifndef __CAMERACOM_H__
#define __CAMERACOM_H__

// 数据包配置
#include "includes.h"
#include "usart.h"
#include "vector.h"
#define CAMERA_RX_BOOL_NUM 0  // 接收 bool 数量
#define CAMERA_RX_BYTE_NUM 1  // 接收 byte 数量
#define CAMERA_RX_SHORT_NUM 0 // 接收 short 数量
#define CAMERA_RX_INT_NUM 0   // 接收 int 数量
#define CAMERA_RX_FLOAT_NUM 4 // 接收 float 数量

#define CAMERA_PREFIX_1 0xFF // 调试器前缀
#define CAMERA_PREFIX_2 0xFE // 调试器前缀
#define CAMERA_SUFFIX_1 0xAA // 调试器后缀
#define CAMERA_SUFFIX_2 0xDD // 调试器后缀

#define LIGHT_PREFIX_1 0xCA
#define LIGHT_PREFIX_2 0xCA
#define LIGHT_SUFFIX_1 0xAA
#define LIGHT_SUFFIX_2 0xBB

#define CAMERA_LIGHT_DATA_LEN 2
#define CAMERA_DATA_LEN 50 // 调试器数据包长度

// 数据包大小
#define CAMERA_RX_PACK_SIZE                                  \
    (((CAMERA_RX_BOOL_NUM + 7) >> 3) + CAMERA_RX_BYTE_NUM +  \
     (CAMERA_RX_SHORT_NUM << 1) + (CAMERA_RX_INT_NUM << 2) + \
     (CAMERA_RX_FLOAT_NUM << 2))

// 调试器状态
typedef enum _camerastate
{
    CAMERA_STATE_RECEIVING_DONE, // 接收完毕
    CAMERA_STATE_WAIT_PREFIX,    // 等待前缀
    CAMERA_STATE_WAIT_SUFFIX,    // 等待后缀
    CAMERA_STATE_RECEIVING_DATA  // 正在接收数据
} CameraStateTypedef;

// 调试器消息结构
typedef struct _cameragermsg
{
    u8 RxData[CAMERA_DATA_LEN];     // 接收数据缓冲区
    u8 Prefix;                      // 前缀
    u8 Suffix;                      // 后缀
    u8 RxDataSize;                  // 接收数据大小
    bool GetSuffix;                 // 是否获取到后缀
    CameraStateTypedef CameraState; // 调试器当前状态
} CameraMsgTypedef;

// 接收数据包结构
typedef struct _camerarxmsgpack
{
#if CAMERA_RX_BOOL_NUM > 0
    u8 Bools[CAMERA_RX_BOOL_NUM]; // 接收布尔值数组
#endif
#if CAMERA_RX_BYTE_NUM > 0
    char Bytes[CAMERA_RX_BYTE_NUM]; // 接收字节数组
#endif
#if CAMERA_RX_SHORT_NUM > 0
    short Shorts[CAMERA_RX_SHORT_NUM]; // 接收 short 数组
#endif
#if CAMERA_RX_INT_NUM > 0
    int Ints[CAMERA_RX_INT_NUM]; // 接收 int 数组
#endif
#if CAMERA_RX_FLOAT_NUM > 0
    float Floats[CAMERA_RX_FLOAT_NUM]; // 接收 float 数组
#endif
    char Space; // 保留空间
    u8 light;
} CameraRxMsgPackTypedef;

typedef struct _CameraLightTxMsg
{
    u8 rxdata[10];
    u8 txdata[10];
    u8 Prefix;                      // 前缀
    u8 Suffix;                      // 后缀
    u8 RxDataSize;                  // 接收数据大小
    bool GetSuffix;   
    CameraStateTypedef CameraState; // 调试器当前状态
    u8 LightState;
    /* data */
}CameraLightMsg;

// 全局变量
extern CameraMsgTypedef CameraMsg;          // 调试器消息
extern CameraRxMsgPackTypedef CameraRxPack; // 接收数据包
extern CameraLightMsg CameraLightPack;
extern u8 CameraLight;
bool Camera_ReceiveHandler(CameraMsgTypedef *CameraMsg, uint8_t data);
bool Camera_NormalizeRxMsg(CameraMsgTypedef *cameraMsg,
                           CameraRxMsgPackTypedef *rxPack);
void Camera_LightReceiveHandler(CameraLightMsg *cmsg,u8 data);
void Camera_LightTransmit(void);
#endif /* __CAMERACOM_H__ */
