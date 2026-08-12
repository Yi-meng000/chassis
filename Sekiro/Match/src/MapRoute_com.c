#include "MapRoute_com.h"

RouteMsgTypedef RouteMsg;
RouteRxMsgPackTypedef RouteRxPack;

bool Route_ReceiveByte(RouteMsgTypedef *routeMsg, u8 data)
{
    if (routeMsg->RouteState == ROUTE_STATE_WAIT_SUFFIX)
    {
        routeMsg->Suffix = data;
        if (routeMsg->Suffix == ROUTE_SUFFIX2)
        {
            routeMsg->RouteState = ROUTE_STATE_RECEIVING_DONE;
            if (!Route_ProcessRxMsg(routeMsg, &RouteRxPack))
                return 0;
            routeMsg->RxDataSize = 0;
        }
        else
        {
            routeMsg->RouteState = ROUTE_STATE_RECEIVING_DONE;
            routeMsg->RxDataSize = 0;
        }
    }
    else if (routeMsg->RouteState == ROUTE_STATE_RECEIVING_DATA)
    {
        routeMsg->Suffix = data;
        if (routeMsg->Suffix == ROUTE_SUFFIX1)
            routeMsg->RouteState = ROUTE_STATE_WAIT_SUFFIX;
        else
        {
            routeMsg->RxData[routeMsg->RxDataSize++] = data;
            if (routeMsg->RxDataSize >= ROUTE_DATA_LEN)
            {
                routeMsg->RouteState = ROUTE_STATE_RECEIVING_DONE;
                routeMsg->RxDataSize = 0;
            }
        }
    }
    else if (routeMsg->RouteState == ROUTE_STATE_RECEIVING_DONE)
    {
        routeMsg->Prefix = data;
        if (routeMsg->Prefix == ROUTE_PREFIX1)
            routeMsg->RouteState = ROUTE_STATE_WAIT_PREFIX;
    }
    else if (routeMsg->RouteState == ROUTE_STATE_WAIT_PREFIX)
    {
        routeMsg->Prefix = data;
        if (routeMsg->Prefix == ROUTE_PREFIX2)
            routeMsg->RouteState = ROUTE_STATE_RECEIVING_DATA;
        else
        {
            routeMsg->RouteState = ROUTE_STATE_RECEIVING_DONE;
            routeMsg->RxDataSize = 0;
        }
    }
    return 1;
}
bool Route_ProcessRxMsg(RouteMsgTypedef *routeMsg, RouteRxMsgPackTypedef *rxPack)
{
#if ROUTE_RX_BYTE_NUM
    memcpy(rxPack, routeMsg->RxData, (routeMsg->RxDataSize) * sizeof(uint8_t));
#endif
    routeMsg->Receive = 1;
    return 1; // TODO
}
void Route_SendMsg(uint8_t *map, RouteMsgTypedef *routeMsg)
{
    uint16_t txIndex = 0;
    // uint8_t sum = 0;

    routeMsg->TxData[txIndex++] = ROUTE_PREFIX1;
    routeMsg->TxData[txIndex++] = ROUTE_PREFIX2;
    for (int i = 0; i < 12; i++)
    {
        routeMsg->TxData[txIndex++] = (map[i] + 0x30);
    }
    routeMsg->TxData[txIndex++] = ROUTE_SUFFIX1;
    routeMsg->TxData[txIndex++] = ROUTE_SUFFIX2;
    if (HAL_UART_Transmit_IT(&huart4, routeMsg->TxData, txIndex) != HAL_OK)
        Error_Handler();
    routeMsg->Transmit = true;
}
