#include "sensorcom.h"

DT_Data2fTypedef DTData1 = {0}, DTData2 = {0};

void SENSOR_RecvHandler(FDCAN_RxHeaderTypeDef rxmsg, uint8_t rxdata[])
{

    switch (rxmsg.Identifier)
    {
        //    case SLAVE_SENSOR_SET_POSTURE:
        //        break;
        //    case SLAVE_SENSOR_REPLY_POSTURE:
        //        re_x = (s16)(rxdata[3] << 8 | rxdata[2]);
        //        re_y = (s16)(rxdata[1] << 8 | rxdata[0]);
        //        re_angle =  (float)(rxdata[7] << 24 | rxdata[6] << 16
        //                                             | rxdata[5] << 8 | rxdata[4]) / 100.f;
        //        if(abs(re_x  - Chassis.ChassisPosReal.x) > 2000 || abs(re_y - Chassis.ChassisPosReal.y) > 2000
        //        || abs(re_angle - Chassis.ChassisPosReal.angle) > 1000.f)
        //        {
        //            SENSOR_SetPos(0x07,Chassis.ChassisPosReal.x,Chassis.ChassisPosReal.y,Chassis.ChassisPosReal.angle);
        //            return;
        //        }
        //        Chassis.ChassisPosReal.x = (s16)(rxdata[1] << 8 | rxdata[0]);
        //        Chassis.ChassisPosReal.y = (s16)(rxdata[3] << 8 | rxdata[2]);
        //        Chassis.ChassisPosReal.angle = (float)(rxdata[7] << 24 | rxdata[6] << 16
        //                                             | rxdata[5] << 8 | rxdata[4]) / 100.f;
        //        break;
        //    case SLAVE_SENSOR_REPLY_SPEED:
        //      Chassis.ChassisPosReal.vx =
        //          (float)((s16)(rxdata[1] << 8 | rxdata[0])) / 10.0f;
        //      Chassis.ChassisPosReal.vy =
        //          (float)((s16)(rxdata[3] << 8 | rxdata[2])) / 10.0f;
        //      Chassis.ChassisPosReal.v =
        //          (float)((s16)(rxdata[5] << 8 | rxdata[4])) / 10.0f;
        //      Chassis.ChassisPosReal.w =
        //          (float)((s16)(rxdata[7] << 8 | rxdata[6])) / 10.0f;
        //      break;
        //    case SLAVE_SENSOR_OPEN_DTSTREAM1:
        //        memcpy(&DTData1.X,rxdata,sizeof(int));
        //        memcpy(&DTData1.Y,rxdata + 4,sizeof(int));
        //        DTData1.GetData = true;
        //        break;
        //    case SLAVE_SENSOR_OPEN_DTSTREAM2:
        //        memcpy(&DTData2.X,rxdata,sizeof(int));
        //        memcpy(&DTData2.Y,rxdata + 4,sizeof(int));
        //        DTData2.GetData = true;
    default:
        break;
    }
}

void SENSOR_SetPos(uint8_t ctrlword, s16 disx, s16 disy,
                   s16 degangle)
{
    //    uint8_t Data[8] = {0};
    //    FDCAN_RxHeaderTypeDef rxheader;

    //    HeaderPrepare(MASTER_SENSOR_SET_POSTURE,8,&rxheader);

    //    // ??????
    //    Data[0] = 'S';
    //    Data[1] = ctrlword;
    //    Data[2] = (uint8_t)(disx);
    //    Data[3] = (uint8_t)(disx >> 8);
    //    Data[4] = (uint8_t)(disy);
    //    Data[5] = (uint8_t)(disy >> 8);
    //    Data[6] = (uint8_t)(degangle * 10);
    //    Data[7] = (uint8_t)(degangle * 10 >> 8);

    //    if(CAN_Queue_IfFull(&CAN1_Txqueue))
    //        Can1FullFlag++;
    //    else
    //        CAN_Enqueue(&CAN1_Txqueue,rxheader,Data);
}

void SENSOR_AskDTData(uint8_t ctrlword, uint8_t times)
{

    //    uint8_t Data[8] = {0};
    //    FDCAN_RxHeaderTypeDef rxheader;

    //    HeaderPrepare(MASTER_SENSOR_ASK_DTSTREAM,2,&rxheader);

    //    Data[0] = ctrlword;
    //    Data[1] = times;

    //    if(CAN_Queue_IfFull(&CAN1_Txqueue))
    //        Can1FullFlag++;
    //    else
    //        CAN_Enqueue(&CAN1_Txqueue,rxheader,Data);
}
