#ifndef __ROBOTCOM_H__
#define __ROBOTCOM_H__
/**
 * @brief       [R1R2LCD通讯协议]
 * 红区梅林编号：0-12     蓝区梅林编号：0-12
 * 武馆
 * +----+----+----+    +----+----+----+
 * |  1 |  2 |  3 |    |  3 |  2 |  1 |
 * +----+----+----+    +----+----+----+
 * |  4 |  5 |  6 |    |  6 |  5 |  4 |
 * +----+----+----+    +----+----+----+
 * |  7 |  8 |  9 |    |  9 |  8 |  7 |
 * +----+----+----+    +----+----+----+
 * | 10 | 11 | 12 |    | 12 | 11 | 10 |
 * +----+----+----+    +----+----+----+
 *
 * 14个有效u8数据：
 *
 * 索引0-12：12个梅花桩上放置的方块信息
 * - 0：无方块
 * - 1：R1方块
 * - 2：R2方块
 * - 3：假方块
 *
 * 索引13：R2夹爪状态标志位（对接时使用）
 * - 0：夹爪闭合
 * - 1：夹爪松开
 *
 * 索引14：R1R2连接状态标志位（对抗区使用）
 * - 0：等待连接
 * - 1：进行连接
 * @date        2026-01-11
 */
#include "includes.h"
#include "Sekiro.h"
#define ROBOTCOM_RX_BYTE_NUM 50 // 接收 byte 数量

#define ROBOTCOM_PREFIX_1 0xFF // 二维码通信前缀
#define ROBOTCOM_PREFIX_2 0xEE // 二维码通信前缀
#define ROBOTCOM_SUFFIX_1 0xAA // 二维码通信后缀
#define ROBOTCOM_SUFFIX_2 0xDD // 二维码通信后缀

#define ROBOTCOM_Z3_PREFIX_1 0xBB
#define ROBOTCOM_Z3_PREFIX_2 0xCC
#define ROBOTCOM_Z3_SUFFIX_1 0x66
#define ROBOTCOM_Z3_SUFFIX_2 0x99

#define ROBOTCOM_DATA_LEN 50 // 二维码通信数据长度
// 二维码通信状态
typedef enum _ROBOTCOMstate
{
    ROBOTCOM_STATE_RECEIVING_DONE, // 接收完毕
    ROBOTCOM_STATE_WAIT_PREFIX,    // 等待前缀
    ROBOTCOM_STATE_WAIT_SUFFIX,    // 等待后缀
    ROBOTCOM_STATE_RECEIVING_DATA  // 正在接收数据
} ROBOTCOMStateTypedef;
enum RoboComPackState
{
    Common = 1,
    NoMatch,
    Invalid_input,
    QRCodeR1_dis = 4,
};
// 通信消息结构
typedef struct _ROBOTCOMgermsg
{
    u8 RxData[ROBOTCOM_DATA_LEN];       // 接收数据缓冲区
    u8 Prefix;                          // 前缀
    u8 Suffix;                          // 后缀
    u8 RxDataSize;                      // 接收数据大小
    bool GetSuffix;                     // 是否获取到后缀
    ROBOTCOMStateTypedef RobotComState; // 二维码通信当前状态
    bool Zone_messageSwitch;
} RobotComMsgTypedef;
typedef struct _RobotComRxMsgStruct
{
    bool receive_effect;
    u8 type;
    u16 payload_len;
    u8 path_len;
    u8 path[8];
    u8 grab_num;
    u8 grab_pos[2];
    u8 map_state[12];
    u8 r1_command[2];

    s16 ToR1_x;
    s16 ToR1_y;
    float ToR1Angle;
    bool pos_receive;
} RobotComRxMsgStruct;

typedef struct _MatchTraceOn
{
    u8 rxdata[10];
    bool Enable;
    bool Start;
    bool Reset;
    bool Side;
    ROBOTCOMStateTypedef RobotComState; // 二维码通信当前状态
    u8 RxDataSize;
    u8 suffix_cnt;
}MatchTraceOn;
#define TraceOn_Prefix 0x65
#define TraceOn_Suffix 0xFF
extern MatchTraceOn MatchTrace;
extern RobotComRxMsgStruct RobotRxmsg;
extern RobotComMsgTypedef RobotComMsg; // 二维码通信消息
bool RobotCom_ReceiveHandler(RobotComMsgTypedef *rmsg, uint8_t data);
bool RobotCom_NormalizeRxMsg(RobotComMsgTypedef *rmsg, RobotComRxMsgStruct *rxPack);
void RobotCom_QrcodeScan(RobotComRxMsgStruct *rxPack, u8 data[]);
void RobotCom_ToActuator(RobotComRxMsgStruct *rxPack);
bool RobotCom_TraceOnReceive(MatchTraceOn *trace,u8 data);
bool RobotCom_TraceRxMsgHandler(MatchTraceOn *trace);
#endif /* __ROBOTCOM_H__ */
