#include "VescMotor.h"
#include "mathFunc.h"
#include "fdcan.h"
#include "FD_Canqueue.h"
VescMOTOR Vescmotor[USE_VESCNUM];
FDCAN_SendQueueType Vesc_queue;
//运用CAN队列进行发送
/**
 * @brief 初始化
 * 
 */
void VescInit()
{

	VecsLimit limit;
    VescValue valset;
    limit.CurrentLimit = 30;
    limit.releaseWhenStuck = true;
    limit.StuckCheck = true;
    limit.timeoutCheck = true;
    valset.current = 0;
    valset.duty = 0.1;
    valset.speed = 0;
    valset.handbrakeCurrent = 5;
    for(int i = 0;i < USE_VESCNUM;i++)
    {
        Vescmotor[i].limit = limit;
        Vescmotor[i].valSet = valset;
        Vescmotor[i].mode= VESC_POSITION;
        Vescmotor[i].Begin = false;
        Vescmotor[i].Enable = false;
        Vescmotor[i].PolePairs = 7;
    //  Vescmotor[i]->argum.anglePre = 0;
        Vesc_queue.FDCAN_DataSend[i].IDE = FDCAN_EXTENDED_ID;
    }
    Vesc_queue.Canx = FDCAN2;
    Vesc_queue.Front = Vesc_queue.Rear = 0;
}
/**
 * @brief 接收CAN通信
 * 
 * @param Rxheader 
 * @param Rx_data 
 */
void VescReceiveData_CAN2(FDCAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data)
{
    int32_t index=0;
    uint8_t id = (Rxheader.Identifier & 0xf) -1;
    
    if(id == 5) // id为5则整定上位机为3.0
    {
        if(Rxheader.IdType == FDCAN_STANDARD_ID && Rxheader.RxFrameType == FDCAN_DATA_FRAME)
        {
            if(Rxheader.Identifier >> 8 == CAN_PACKET_STATUS) //读电流和速度
            {
                Vescmotor[id].valNow.speed = buffer_32_to_float(Rx_data,1e0,&index) / Vescmotor[id].PolePairs;//真实速度为读取数据除以极对数
                Vescmotor[id].valNow.current = buffer_16_to_float(Rx_data,1e1,&index);//固定的数值
            }
            if(Rxheader.Identifier >>8 == CAN_PACKET_STATUS_4)
            {
                index+=6;//直接读角度
                Vescmotor[id].valNow.angle = buffer_16_to_float(Rx_data,5e1,&index);
            }
            Vescmotor[id].Begin = 1;
        }
    }
    else //以下为 0.9版本整定
    {
        if(Rxheader.Identifier >> 8 == CAN_PACKET_STATUS)
        {
            Vescmotor[id].valNow.speed = buffer_32_to_float(Rx_data,1e0,&index)/ Vescmotor[id].PolePairs;
            Vescmotor[id].valNow.current = buffer_16_to_float(Rx_data,1e1,&index);
            Vescmotor[id].valNow.angle = buffer_16_to_float(Rx_data,1e1,&index);
            //决定性变量 speed、current、angle
            ChangeDataByte(&Rx_data[6],&Rx_data[7]);//高低位转换
            DecodeU16Data(&Vescmotor[id].argum.angleNow,&Rx_data[6]);

            Vescmotor[id].argum.distance = Vescmotor[id].argum.angleNow -Vescmotor->argum.anglePre;
            Vescmotor[id].argum.anglePre = Vescmotor[id].argum.angleNow;

            if(ABS(Vescmotor[id].argum.distance) > 1800)
                Vescmotor[id].argum.distance -= 3600;//判断是正转还是反转
            Vescmotor[id].argum.position += Vescmotor[id].argum.distance;
            Vescmotor[id].valNow.angleABS = Vescmotor[id].argum.position/ 10.0;
            //distance、position、angleABS做记录的变量 无决定性作用
            Vescmotor[id].Begin = 1;
            //收到CAN则使能begin
        }
    }
    Vescmotor[id].argum.lastRxTime = 0;//收到则超时计数位清零
}
void VescPosition_Mode(uint8_t controlID,float position){
    int32_t sendIndex = 0;

    if(CAN_Queue_IfFull(&Vesc_queue)){
        return;
    }
    buffer_append_int32(Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].Data,(uint32_t)(position * 1e6f),&sendIndex);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].ID = 0xf0000000 | controlID | ((uint32_t)CAN_PACKET_SET_POS << 8);
    // 发送ID组成
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].DLC = sendIndex;
    Vesc_queue.Rear = (Vesc_queue.Rear+1) % FDCAN_QUEUESIZE;
    //相对简单 把想达到的变量传出去就行
}
void VescRPM_Mode(uint8_t controlID,float speed)
{
    int32_t sendIndex = 0;
    
    if(CAN_Queue_IfFull(&Vesc_queue)){
        return;
    }
    buffer_append_int32(Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].Data,speed,&sendIndex);
    //将浮点数赋到message中，index自动加
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].ID = 0xf0000000 | controlID | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].DLC = sendIndex;
    Vesc_queue.Rear = (Vesc_queue.Rear+1) % FDCAN_QUEUESIZE;

}

void VescCurrent_Mode(uint8_t controlID,float current)
{
    int32_t sendIndex = 0;
    
    if(CAN_Queue_IfFull(&Vesc_queue)){
        return;
    }
    buffer_append_int32(Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].Data,(int32_t)(current * 1e3f),&sendIndex);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].ID = 0xf0000000 | controlID | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].DLC = sendIndex;
    Vesc_queue.Rear = (Vesc_queue.Rear+1) % FDCAN_QUEUESIZE;

}
void VescDuty_Mode(uint8_t controlID,float duty)
{
    int32_t sendIndex = 0;
    
    if(CAN_Queue_IfFull(&Vesc_queue)){
        return;
    }
    buffer_append_int32(Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].Data,(uint32_t)(duty * 100000),&sendIndex);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].ID = 0xf0000000 | controlID | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].DLC = sendIndex;
    Vesc_queue.Rear = (Vesc_queue.Rear+1) % FDCAN_QUEUESIZE;

}
void VescBrake_Mode(uint8_t controlID,float handbrake)
{
    int32_t sendIndex = 0;
    
    if(CAN_Queue_IfFull(&Vesc_queue)){
        return;
    }
    buffer_append_int32(Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].Data,(uint32_t)(handbrake * 1e3f),&sendIndex);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].ID = 0xf0000000 | controlID | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8);
    Vesc_queue.FDCAN_DataSend[Vesc_queue.Rear].DLC = sendIndex;
    Vesc_queue.Rear = (Vesc_queue.Rear+1) % FDCAN_QUEUESIZE;

}
/**
 * @brief 检查是否CAN通信超时
 * 
 * @param id 
 */
void VescTimeOut(uint8_t id)
{
    if(Vescmotor[id].argum.lastRxTime++ >1000 && Vescmotor[id].Enable)
    {
        if(Vescmotor[id].argum.TimeoutTick++ > 50 )
        {
            Vescmotor[id].statusflag.timeoutflag = 1;
        }
        else
            Vescmotor[id].statusflag.timeoutflag = 0;
    }
}
/**
 * @brief 检查是否堵转
 * 
 * @param id 
 */
void VescStuck_Check(uint8_t id)
{
    if(Vescmotor[id].valNow.current > 45 && Vescmotor[id].Enable)
    {
        if(Vescmotor[id].argum.stuckCount++ > 3000)
        {
            Vescmotor[id].statusflag.stuckflag = 1;
            // Answer_queue.messages[Answer_queue.tail].DLC = 2;
            // Answer_queue.messages[Answer_queue.tail].id = 0x020201EE;
            // Answer_queue.messages[Answer_queue.tail].data[0] = id+1 ;
            // Answer_queue.messages[Answer_queue.tail].data[1] = 2;
            // Answer_queue.tail = (Answer_queue.tail + 1) % FDCAN_QUEUESIZE;
            if(Vescmotor[id].limit.releaseWhenStuck)
                Vescmotor[id].Enable = 0;
        }
        else
            Vescmotor[id].statusflag.timeoutflag = 0;
    }
}
void VescFunc()
{
    for(int i = 0;i < USE_VESCNUM ;i++)
    {
        if(Vescmotor[i].Enable)
        {   
            if(Vescmotor[i].limit.timeoutCheck)
                VescTimeOut(i);
            if(Vescmotor[i].limit.StuckCheck)
                VescStuck_Check(i);
            if(Vescmotor[i].Begin)
            {
                switch (Vescmotor[i].mode)
                {
                    case VESC_RPM:
                        VescRPM_Mode(i+1,Vescmotor[i].valSet.speed * Vescmotor[i].PolePairs);//输入速度等于速度*极对数
                        break;
                    case VESC_POSITION:
                        VescPosition_Mode(i+1,Vescmotor[i].valSet.angle);
                        break;
                    case VESC_CURRENT:
                        VescCurrent_Mode(i+1,Vescmotor[i].valSet.current);
                        break;
                    case VESC_DUTY:
                        VescDuty_Mode(i+1,Vescmotor[i].valSet.duty);
                        break;
                    case VESC_HANDBRAKE:
                        VescBrake_Mode(i+1,Vescmotor[i].valSet.handbrakeCurrent);
                        break;
                    default:
                        break;
                }
            }
            else 
                VescBrake_Mode(i+1,Vescmotor[i].valSet.handbrakeCurrent);

        }
        else
            VescCurrent_Mode(i+1,0.0);
    }

}
