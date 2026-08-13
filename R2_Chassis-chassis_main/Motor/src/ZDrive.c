#include "ZDrive.h"
#include "stdbool.h"
#include "stdlib.h"
#include "FD_Canqueue.h"
#include "mathFunc.h"
#define CHASSIS_STEER_RATIO 4.2f
#define CHASSIS_DRIVE_RATIO 3.f
FDCAN_SendQueueType *ZdriveSendQueue;
Zdrive Zmotor[USE_ZDRIVE_NUM];
void ZdriveInit()
{
	  ZdriveSendQueue = &CAN2_Txqueue;
    for(int i = 0; i < USE_ZDRIVE_NUM ;i++)
    {
				Zmotor[i].param.GearRatio = 1.f;
//				if(i < 4)
//				   Zmotor[i].param.ReductionRatio = CHASSIS_STEER_RATIO;
//				else if(i >=4 && i < 8)
//				   Zmotor[i].param.ReductionRatio = CHASSIS_DRIVE_RATIO;
//				else
				Zmotor[i].param.ReductionRatio = 1.f;
				Zmotor[i].valSetPre.angle = 1e5;
        Zmotor[i].valSetNow.speed = 0;
        Zmotor[i].valSetNow.angle = 0;
        Zmotor[i].statusflag.Arriveflag =false;
        Zmotor[i].argum.GapCnt = 0;
        Zmotor[i].valReal.angle = 0;
        Zmotor[i].param.kd = 10.f;
        Zmotor[i].param.kp = 2.3f;
        Zmotor[i].mode = Zdrive_Postion;
        Zmotor[i].Enable = 0;
        Zmotor[i].Begin = 0;
        Zmotor[i].statusflag.ZeroPoint = 0;
        Zmotor[i].statusflag.Zeroflag = 0;
			
				//ZdriveSet(0,i+1,Pur);	
    }
		

}
void ZdriveSet(float data,uint8_t id,uint8_t set_code)
{
    if(CAN_Queue_IfFull(ZdriveSendQueue)){
				Can2FullFlag++;
        return;
    }
    if(set_code == PosIn || set_code == Pur)
    {
        data /= (360.f / Zmotor[id- 1].param.ReductionRatio);
    }
    else if(set_code == VelIn)
    {
        data /= (60.f / Zmotor[id - 1].param.ReductionRatio);
    }
		if(id == 0)
			id = 0xF;
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID = id | (set_code << 4);
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 4;
    memcpy(ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].Data,&data,sizeof(float));
    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;
}
void ZdriveReceive(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_Data)
{
    uint8_t controlID =Rxheader.Identifier & 0xF;
    uint8_t operationID = Rxheader.Identifier >> 4;
    float tmp_pos = 0;
    int16_t tmp_vel = 0,tmp_cur = 0;
    if(Rxheader.DataLength  == 4)
    {
        switch (operationID)
        {
        case Pur: //position
        {   
					  Zmotor[controlID -1].valPre.angle = Zmotor[controlID -1].valReal.angle;
            memcpy(&Zmotor[controlID - 1].valReal.angle,Rx_Data,sizeof(float));

            Zmotor[controlID -1].valReal.angle *= (360.f / Zmotor[controlID - 1].param.ReductionRatio);
            break;
        }
        case Cur_M: //current
        {   
            memcpy(&Zmotor[controlID - 1].valReal.current,Rx_Data,sizeof(float));
            Zmotor[controlID - 1].valReal.Torque =  Zmotor[controlID -1].valReal.current;
            break;
        }
        case Vel: //speed
        {
            memcpy(&Zmotor[controlID -1].valReal.speed,Rx_Data,sizeof(float));
            Zmotor[controlID - 1].valReal.speed *= (60.f / Zmotor[controlID - 1].param.ReductionRatio);
            break;
        }
        case Mode://mode
        {
            float tempMode;
            memcpy(&tempMode,Rx_Data,sizeof(float));
            Zmotor[controlID -1].modeRead = (ZdriveMode)tempMode;
            break;
        }
        case Err: //error
        {
            float tempErr;
            memcpy(&tempErr,Rx_Data,sizeof(float));
            Zmotor[controlID - 1].statusflag.err = (ZdriveErr)tempErr;
            break;					
        }
        case PosIn://pos_in
        {
            float tempPos_in;
            memcpy(&tempPos_in,Rx_Data,sizeof(float));
            Zmotor[controlID - 1].valReal.posIn = tempPos_in*360.f/Zmotor[controlID-1].param.ReductionRatio;
            break;
        }
        case Vel_Limit:
        {
            float tempVelLimit;
            memcpy(&tempVelLimit, Rx_Data, sizeof(float));
            Zmotor[controlID - 1].valReal.velLimit = tempVelLimit;
            break;
        }
        case Acc_Acu:
        {
            float tempAccAcu;
            memcpy(&tempAccAcu, Rx_Data, sizeof(float));
            Zmotor[controlID - 1].valReal.accAcu = tempAccAcu;
            break;
        }
        case Acc_Dec:
        {
            float tempAccDec;
            memcpy(&tempAccDec, Rx_Data, sizeof(float));
            Zmotor[controlID - 1].valReal.accDec = tempAccDec;
            break;
        }
        default:
            break;
        }
    }
    else if(Rxheader.DataLength  == 8)
    {
        memcpy(&tmp_pos,Rx_Data,sizeof(float));
        
        Zmotor[controlID -1].valPre.angle = Zmotor[controlID -1].valReal.angle;
        Zmotor[controlID -1].valReal.angle = ((tmp_pos)/(float) 0xffffffff * (POU - POD) + POD) / Zmotor[controlID - 1].param.ReductionRatio;

        memcpy(&tmp_vel,Rx_Data + 4,sizeof(int16_t));
        Zmotor[controlID - 1].valReal.speed = (float)(tmp_vel / (float)0xffff * (2 * Velocity_Limit) -Velocity_Limit);

         memcpy(&tmp_cur,Rx_Data+6,sizeof(int16_t));
        Zmotor[controlID - 1].valReal.current = (float)(tmp_cur) / (float)0xffff * (2 * Current_Limit) - Current_Limit;
    }
    
}


void ZdriveAsk(uint8_t id,uint8_t ask_code){

    if(CAN_Queue_IfFull(ZdriveSendQueue)){
		Can2FullFlag++;
        return;
    }
    if(id == 0)
        id = 0xF;
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID = id | (ask_code << 4);
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 0;
    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;

}
void ZdriveSetPVT(float speed,float angle,uint8_t id)
{
    uint8_t data[8] = {0};
    uint32_t vel_mod_u32 = (uint32_t)((speed + Vel_Limit) / (2.0f * Vel_Limit) * (float)0xffffffff);
    uint32_t pos_mod_u32 = (uint32_t)(((angle - POD) / (POU - POD)) * (float)0xffffffff);
    memcpy(data,&vel_mod_u32,sizeof(uint32_t));
    memcpy(data+4,&pos_mod_u32,sizeof(uint32_t));

    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID = id | ( PVT_Frame << 4);
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 0;
    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;

}

void ZdriveSetPID(float value, uint8_t id, uint8_t pid_code)
{
    uint8_t data[8] = {0};
    uint32_t val_u32;

    // 将参数转换为 0~0xffffffff
    val_u32 = (uint32_t)(value / 100.0f * (float)0xffffffff);

    memcpy(data, &val_u32, sizeof(uint32_t));

    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID  = id | (pid_code << 4);
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 8;

    memcpy(ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].Data, data, 8);

    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;
}

void ZdriveSetPosVelLimit(float vel_limit, uint8_t id)
{
    uint8_t data[8] = {0};

    memcpy(data, &vel_limit, sizeof(float));

    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID  = id | (Vel_Limit << 4);
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 4;

    memcpy(ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].Data, data, sizeof(uint8_t) * 4);

    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;
		ZdriveAsk(id, Vel_Limit);


}
void ZdriveSetAccel(float ace, uint8_t id)
{
    uint8_t data[8] = {0};
    uint32_t vel_u32;

    memcpy(data, &ace, sizeof(float));

    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID  = id | (Acc_Acu << 4);
    ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 4;

    memcpy(ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].Data, data, sizeof(uint8_t) * 4);

    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;
		
		ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].ID  = id | (Acc_Dec << 4);
		ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].DLC = 4;
		
    memcpy(ZdriveSendQueue->FDCAN_DataSend[ZdriveSendQueue->Rear].Data, data, sizeof(uint8_t) * 4);

    ZdriveSendQueue->Rear = (ZdriveSendQueue->Rear + 1) % FDCAN_QUEUESIZE;

		ZdriveAsk(id, Acc_Acu);
    ZdriveAsk(id, Acc_Dec);

		
}

void ZdriveFunc()
{
   for(int i = 0;i < USE_ZDRIVE_NUM ;i++)
    {
				if(i >= 8)
						ZdriveSendQueue = &CAN3_Txqueue;
				else
						ZdriveSendQueue = &CAN2_Txqueue;
        if(Zmotor[i].Enable)
        {

            if(Zmotor[i].Begin)
            {
                switch (Zmotor[i].mode)
                {
                    case Zdrive_Speed:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Speed)
                        {
                            ZdriveSet((float)Zdrive_Speed,i+1,Mode);
                            ZdriveAsk(i+1,Mode);
                        }
                        else
												{
													if(i < 8)
													{
															if(fabs(Zmotor[i].valSetNow.speed) < 0.5f && fabs(Zmotor[i].valSetPre.speed) >= 0.5f)
																ZdriveSet(0,i+1,PID_VEL_I);
															else if(fabs(Zmotor[i].valSetNow.speed) >= 0.5f && fabs(Zmotor[i].valSetPre.speed) < 0.5f)
																ZdriveSet(0.8f,i+1,PID_VEL_I);
													}
													ZdriveSet(Zmotor[i].valSetNow.speed,i+1,VelIn);
													Zmotor[i].valSetPre.speed = Zmotor[i].valSetNow.speed;
												}	
												
                        break;   
                    }
                    case Zdrive_Current:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Current)
                        {
                            ZdriveSet((float)Zdrive_Current,i+1,Mode);
                            ZdriveAsk(i+1,Mode);
                        }
                        else
                            ZdriveSet(Zmotor[i].valSetNow.speed,i+1,CurIn);
                        break;
                    }
                    case Zdrive_Disable:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Disable)
                        {
                            ZdriveSet((float)Zdrive_Disable,i+1,Mode);
                            ZdriveAsk(i+1,Mode);
                        }
                        break;
                    }
                    case Zdrive_Postion:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Postion)
                        {
                            ZdriveSet((float)Zdrive_Postion,i+1,Mode);
                            ZdriveAsk(i+1,Mode);
                        }
                        else
                        {
                            if(!Zmotor[i].pvtparam.PVTflag)
                            {
                                if(fabs((Zmotor[i].valSetNow.angle - Zmotor[i].valSetPre.angle)) > 0.01f)
                                {
                                    //当角度的偏差过大
                                    Zmotor[i].valSetPre.angle = Zmotor[i].valSetNow.angle;
                                    ZdriveSet(Zmotor[i].valSetNow.angle,i+1,PosIn);
                                }
                                else if(Zmotor[i].valSetNow.angle == 0 && fabs(Zmotor[i].valReal.posIn) > 0.5f )
                                {
                                    ZdriveSet(Zmotor[i].valSetNow.angle,i+1,PosIn);
                                }
																		
                            }
                            else
                            {
                                ZdriveSetPVT(Zmotor[i].valSetNow.speed,Zmotor[i].valSetNow.angle,i+1);
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
            
        }
        else
        {
            if(Zmotor[i].modeRead != Zdrive_Disable)
            {
                ZdriveSet((float)Zdrive_Disable,i+1,Mode);
								ZdriveAsk(i+1,Mode);
            }
        }
    }
		ZdriveSendQueue = &CAN2_Txqueue;
    ZdriveAsk(0,Pur);
		ZdriveAsk(0,PosIn);
		ZdriveSendQueue = &CAN3_Txqueue;
    ZdriveAsk(0,Pur);
		ZdriveAsk(0,PosIn);
//		for(int id = 7;id <14;id++){
//		ZdriveAsk(id, Acc_Acu);
//    ZdriveAsk(id, Acc_Dec);
//		ZdriveAsk(id, Vel_Limit);
//		};
}
