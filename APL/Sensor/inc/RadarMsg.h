#ifndef __RADARMSG_H__
#define __RADARMSG_H__

#include <stdint.h>
#include <stdbool.h>
#include "kalman.h"
#include "usart.h"
#ifndef u8
typedef uint8_t u8;
#endif
#define Laser_RX_BOOL_NUM 0
#define Laser_RX_BYTE_NUM 1
#define Laser_RX_SHORT_NUM 0
#define Laser_RX_INT_NUM 0
#define Laser_RX_FLOAT_NUM 5

#define Laser_TX_BYTE_NUM 1
#define Laser_TX_FLOAT_NUM 4

#define Cluster_RX_BOOL_NUM 0
#define Cluster_RX_BYTE_NUM 0
#define Cluster_RX_SHORT_NUM 0
#define Cluster_RX_INT_NUM 0
#define Cluster_RX_FLOAT_NUM 12

#define HEAD_LEN 2

#define DEBUG_PREFIX_1 0XFF
#define DEBUG_PREFIX_2 0XFE

#if HEAD_LEN == 2
#define DEBUG_SUFFIX_1 0XAA
#define DEBUG_SUFFIX_2 0XDD
#endif

#define SENSOR_DATA_LEN 200

#define LASER_PACK_SIZE 25
#define LASER_RXBUFF 50

#define CLUSTER_PACK_SIZE 49
extern __RAM_D2_ ALIGN_32B uint8_t Radar_RxBuff[2][LASER_PACK_SIZE];
/*----------------------------------typedef-----------------------------------*/
typedef enum _debug_state
{
    DEBUG_WAIT_PREFIX,
    DEBUG_RECEIVING_DATA,
} DebugStateTypedef;

typedef struct
{
    // u8 RxData[SENSOR_DATA_LEN];
    u8 TxData[LASER_PACK_SIZE];
    u8 prefix[HEAD_LEN];
    u8 suffix[HEAD_LEN];
    u8 RxDataSize;
    // u8 TxDataSize;
    bool getPrefix;
    bool getSuffix;
    DebugStateTypedef DebugState;
    int16_t offset_x;
    int16_t offset_y;
    float offset_angle;
    bool receive;
} SENSORUSART_MSG;

typedef struct RX_PACK
{
#if Laser_RX_BOOL_NUM > 0
    u8 bools[Laser_RX_INT_NUM];
#endif
#if Laser_RX_BYTE_NUM > 0
    char bytes[Laser_RX_BYTE_NUM];
#endif
#if Laser_RX_SHORT_NUM > 0
    short shorts[Laser_RX_INT_NUM];
#endif
#if Laser_RX_INT_NUM > 0
    int ints[Laser_RX_INT_NUM];
#endif
#if Laser_RX_FLOAT_NUM > 0
    float floats[Laser_RX_FLOAT_NUM];
#endif
    char space;
} LASER_RXPACK;

typedef struct
{
#if Cluster_RX_BOOL_NUM > 0
    u8 bools[Cluster_RX_BOOL_NUM];
#endif
#if Cluster_RX_BYTE_NUM > 0
    char bytes[Cluster_RX_BYTE_NUM];
#endif
#if Cluster_RX_SHORT_NUM > 0
    short shorts[Cluster_RX_SHORT_NUM];
#endif
#if Cluster_RX_INT_NUM > 0
    int ints[Cluster_RX_INT_NUM];
#endif
#if Cluster_RX_FLOAT_NUM > 0
    float floats[Cluster_RX_FLOAT_NUM];
#endif
    char space;
} CLUSTER_RXPACK;

typedef struct
{
    u8 type;
    LASER_RXPACK Laser_RxPack;
    CLUSTER_RXPACK Cluster_RxPack;
} SENSOR_RXPACK;
typedef struct
{
#if Laser_TX_BYTE_NUM > 0
    char bytes[Laser_RX_BYTE_NUM];
#endif
#if Laser_TX_FLOAT_NUM > 0
    float floats[Laser_RX_FLOAT_NUM];
#endif
    char space;
} SENSOR_TXPACK;

typedef enum
{
    RELOC_ORIGIN,
    RELOC_RESET,
} RELOCMODE;

/*----------------------------------variable----------------------------------*/
extern SENSORUSART_MSG SensorUsart_Msg;
extern SENSOR_RXPACK Sensor_RxPack;
extern SENSOR_TXPACK Sensor_TxPack;
/*----------------------------------function----------------------------------*/

void SENSOR_USART_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);
void LaserUsartDeal(SENSOR_RXPACK *Sensor_RxPack);
void LaserRelocation(SENSORUSART_MSG *msg, SENSOR_TXPACK *txPack, uint8_t reloc_mode, float tmp_x, float tmp_y, float tmp_angle);
#endif /* __RADARMSG_H__ */
