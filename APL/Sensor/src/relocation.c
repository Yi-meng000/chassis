#include "relocation.h"

DT_Data2fTypedef ErrData = {0};

void _DT_DealDTData(DT_Data2fTypedef *transdtdata)
{
    float tempx = 0, tempy = 0;

    tempx = FIELD_XDISTENSE -
            ((float)(DTData2.X)) *
                cosf(DEG2RAD(Chassis.ChassisPosReal.angle)) +
            ((float)(DTData2.Y)) *
                sinf(DEG2RAD(Chassis.ChassisPosReal.angle));
    tempy = FIELD_YDISTENSE -
            ((float)(DTData2.X)) *
                sinf(DEG2RAD(Chassis.ChassisPosReal.angle)) +
            ((float)(DTData2.Y)) *
                cosf(DEG2RAD(Chassis.ChassisPosReal.angle));

    // tempx = FIELD_XDISTENSE - (float)(DTData2.X);
    // tempy = FIELD_YDISTENSE - (float)(DTData2.Y);

    transdtdata->X = ((s16)(tempx));
    transdtdata->Y = ((s16)(tempy));
}
void SENSOR_Relocation(uint8_t ctrlword, uint8_t times, uint8_t relocmode, uint8_t zone)
{
    uint8_t X_cnt = 0, Y_cnt = 0;
    uint8_t DTcnt = 0;

    DT_Data2fTypedef DealDTdata = {0};
    DT_DataTypedef beforeReloData = {0};

    TickType_t timeStart, timeNow;
    ErrData.X = 0;
    ErrData.Y = 0;
    DTData1.GetData = 0;
    DTData2.GetData = 0;
    SENSOR_AskDTData(ctrlword, times);

    while (!DTData2.GetData)
        osDelay(1);
    timeStart = xTaskGetTickCount();

    beforeReloData.X = Chassis.ChassisPosReal.x;
    beforeReloData.Y = Chassis.ChassisPosReal.y;

    Chassis.ChassisPosSet.x = Chassis.ChassisPosReal.x;
    Chassis.ChassisPosSet.y = Chassis.ChassisPosReal.y;

    if (times != 0)
    {
        while (DTcnt < times)
        {
            if (DTData2.GetData)
            {
                DTData1.GetData = false;
                DTData2.GetData = false;

                DTcnt++;
                _DT_DealDTData(&DealDTdata);

                if (ABS((DealDTdata.X - beforeReloData.X)) < DT_RELOCCATION_OFFSET_THRESHOLD)
                {
                    X_cnt++;
                    ErrData.X += DealDTdata.X - beforeReloData.X;
                }
                if ((ABS(DealDTdata.Y - beforeReloData.Y) < DT_RELOCCATION_OFFSET_THRESHOLD))
                {
                    Y_cnt++;
                    ErrData.Y += DealDTdata.Y - beforeReloData.Y;
                }
            }
            timeNow = xTaskGetTickCount();
            if (timeNow - timeStart > 5000)
            {
                DTcnt = 0;
                break;
            }
            osDelay(2);
        }
    }
    else
    {
        while (1)
        {
            if (DTData1.GetData && DTData2.GetData)
            {
                DTData1.GetData = false;
                DTData2.GetData = false;
                _DT_DealDTData(&DealDTdata);

                if (fabs(DealDTdata.X - beforeReloData.X) < DT_RELOCCATION_OFFSET_THRESHOLD)
                {
                    X_cnt++;
                    ErrData.X += DealDTdata.X - beforeReloData.X;
                }
                if (fabs(DealDTdata.Y - beforeReloData.Y) < DT_RELOCCATION_OFFSET_THRESHOLD)
                {
                    Y_cnt++;
                    ErrData.Y += DealDTdata.Y - beforeReloData.Y;
                }
            }
            timeNow = xTaskGetTickCount();

            if (timeNow - timeStart > 1000)
            {
                SENSOR_AskDTData(0x00, times);
                break;
            }
            osDelay(1);
        }
    }

    if (X_cnt != 0 || Y_cnt != 0)
    {
        switch (relocmode)
        {
        case RelocX:
            if (X_cnt != 0)
                Chassis.ChassisPosSet.x += (s16)(ErrData.X / X_cnt);
            SENSOR_SetPos(0x04, Chassis.ChassisPosSet.x, Chassis.ChassisPosSet.y, Chassis.ChassisPosReal.angle);
            break;
        case RelocY:
            if (Y_cnt != 0)
                Chassis.ChassisPosSet.y += (s16)(ErrData.Y / Y_cnt);
            SENSOR_SetPos(0x02, Chassis.ChassisPosSet.x, Chassis.ChassisPosSet.y, Chassis.ChassisPosReal.angle);
            break;
        case RelocXY:
            if ((X_cnt != 0) && (Y_cnt != 0))
            {
                Chassis.ChassisPosSet.x += (s16)(ErrData.X / X_cnt);
                Chassis.ChassisPosSet.y += (s16)(ErrData.Y / Y_cnt);
            }
            SENSOR_SetPos(0x06, Chassis.ChassisPosSet.x, Chassis.ChassisPosSet.y, Chassis.ChassisPosReal.angle);
        default:
            break;
        }
    }
}
