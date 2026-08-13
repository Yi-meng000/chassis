#include "uart_comm.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"


//MasterParam Master;
//OffsetMsgStruct OffSetMsg;
void uart_comm_init(uart_comm *ucom,UART_HandleTypeDef *huart)
{
    ucom->write = 0;
    ucom->get_head = 0;
    ucom->huart=huart;
}
/**
 * @brief 接收函数
 * 
 * @param ucom 一个串口结构体的指针
 * @param c 所收到的信息，包括包头包尾 、id 数据和校验位的所有，大小是一个字节
 */
void uart_comm_receive(uart_comm *ucom,unsigned char c)
{
    if (ucom->get_head)    //初始时get_head等于 0 ；
    {
        ucom->tail[0]=ucom->tail[1];
        ucom->tail[1]=c;

        if (ucom->tail[0] == TAIL_CHAR1 && ucom->tail[1] == TAIL_CHAR2)//判断是否收到了包尾
        {   
            //Uchar2Struct(ucom);
            //uart_comm_deal(ucom);
            ucom->write = 0;
            ucom->get_head = 0;
        }
				else
				{
						ucom->rx_data[ucom->write] = c;
						ucom->write++;
				}
    }
    
    else
    {
        ucom->head[0] = ucom->head[1];
        ucom->head[1] = c;
        if (ucom->head[0] == HEAD_CHAR1 && ucom->head[1] == HEAD_CHAR2)
        {
            ucom->get_head = 1;
        }
        
    }
	if (ucom->write >= UART_DATA_SIZE)
    {
        ucom->write = 0;
        ucom->get_head = 0;
    }
}

/**
 * @brief 检验函数
 * 
 * @param ucom 指向一个串口结构体的指针
 * @return true 校验通过
 * @return false 校验未通过
 */
bool uart_comm_detection(uart_comm *ucom)
{
    unsigned short sum = 0;
    int i;
    for ( i = 0; i < ucom->write-2; i++)
    {
        sum+=ucom->rx_data[i];
        sum&=0x00ff;//位与操作，仅保留sum的低8位
    }
    if (sum!=ucom->rx_data[ucom->write-1]) return false;  //计算值与接收到的sum值相比较，相等即校验通过
    return true;
}

/**
 * @brief 发送函数
 * 
 * @param ucom 指向一个串口结构体的指针
 * @param id 储存在包头之后
 * @param data 要发送的一个数据数组，数组内每个元素大小均为一个字节， 0~255之间
 * @param size 数组data内的元素个数
 */
void uart_comm_transmit(uart_comm *ucom,char id,char *data,int size)
{
    ucom->tx_data[0] = HEAD_CHAR1;
    ucom->tx_data[1] = HEAD_CHAR2;
    ucom->tx_data[2] = id;
    unsigned short sum=0;
    uint16_t i;
    for ( i = 3; i < size + 3; i++)
    {
        ucom->tx_data[i]=data[i-3];
        sum+=ucom->tx_data[i];
        sum&=0x00ff;            
    }
    ucom->tx_data[i++]=sum;//校验位
    ucom->tx_data[i++]=TAIL_CHAR1;
    ucom->tx_data[i++]=TAIL_CHAR2;
    if(HAL_UART_Transmit_DMA(ucom->huart,ucom->tx_data,i) != HAL_OK)
			Error_Handler();
			//DMA：直接寄存器访问
}
//  void uart_comm_deal(uart_comm *ucom)
//  {
//      switch (ucom->rx_data[0])
//      {
//      case 0:
//          break;
//      case 1:
//          if (ucom->rx_data[1]==1)
//            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//          else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//          uart_comm_transmit(ucom,0,&ucom->rx_data[1],1);   //将接收到的信息再发送出去
//          break;
//      }
//  }

//float yaw_last = 0, yaw_now = 0;
//short n = 0;
//void G_output_infoSet()
//{

//	yaw_last = yaw_now;
//	yaw_now = g_output_info.yaw;
//	if (yaw_now - yaw_last < -100) // ������ͻ��
//		n += 1;
//	else if (yaw_now - yaw_last > 100)
//		n -= 1;
//	g_output_info.yaw += (float)n * 360;
//}
//void YesenseUc_Init(Yes_comm *ycom,UART_HandleTypeDef *huart)
//{
//    ycom->get_head = 0;
//    ycom->write = 0;
//    ycom->huart = huart;
//    ycom->rx_data[0] = 0x59;
//    ycom->rx_data[1] = 0x53;

//}
//void YesenseUc_receive(Yes_comm *ycom,unsigned char c)
//{
//    if (ycom->Rx_flag == 1)
//    {
//        ycom->write++;
//        ycom->rx_data[ycom->write - 1] = c;
//        if (ycom->write > 97)
//        {
//            ycom->write = 2;
//            ycom->Rx_flag = 0;
//        }
//    }

//    if (ycom->get_head == 1)
//    {
//        if (c == 0x53) 
//        {
//            ycom->write -= 2;
//            uint8_t res = analysis_data(ycom->rx_data, ycom->write);
//            ycom->write = 2;
//            ycom->Rx_flag = 1;
//            ycom->get_head = 0;
//            if (res == 0 || res == 1)
//                G_output_infoSet();
//        }
//        else if (c != 0x59)		
//        ycom->get_head = 0;
//    }
//		
//    if (c == 0x59) 
//    {
//        ycom->get_head = 1;
//    }
//}
///**
// * @brief 将收到的数据转成相应的StructMsg
// * 
// * @param ucom 
// */

//void Uchar2Struct(uart_comm *ucom) 
//{
//    switch (ucom->rx_data[0])
//    {
//    case 1://ControlMsg
//        memcpy(&Master.ControlMsg,&ucom->rx_data[1],sizeof(ControlMsgStruct));
//        Control_Handle();
//        break;
//    case 2://TrotUI 
//        memcpy(&Master.TrotMsg,&ucom->rx_data[1],sizeof(TrotMsgStruct));
//        TrotMsg_Handle();
//		break;
//    case 3:
//        memcpy(&Bound,&ucom->rx_data[1],sizeof(BoundSet));
//        Bezier_Bound_Update();
//		break;
//    case 4:
//        memcpy(&Master.JoyStickMsg,&ucom->rx_data[1],sizeof(JoystickMsgStruct));
//		break;
//    case 5:
//        memcpy(&OffSetMsg.TrotOffset,&ucom->rx_data[1],sizeof(TrotMsgStruct));
//        break;
//    case 6:
//        memcpy(&OffSetMsg.BoundOffset,&ucom->rx_data[1],sizeof(BoundSet));
//        break;
//    case 7:
//        memcpy(&Leg[ucom->rx_data[1]].LegTrot,&ucom->rx_data[1],sizeof(AllFreeStruct));
//        break;
//    default:
//        break;
//    }
//    Master.ReceiveControlMsgFlag = 1;
//	Master.RobotMsg.ReceiveMessage++;
//    
//}
///**
// * @brief 
// * 
// * @param ucom 
// * @param MsgType 11 为 RobotMsg 
// */
//void Struct2Uchar(uart_comm *ucom,uint8_t MsgType)
//{
//    uint16_t size = sizeof(RobotMsgStruct);
//    unsigned char *ch = (unsigned char *)malloc(size * sizeof(unsigned char));
//    if (ch == NULL) 
//        return;
//    RobotMsg_Attain();
//    memcpy(ch, &Master.RobotMsg,size);
//    uart_comm_transmit(ucom,MsgType,ch,size);
//    free(ch);
//}
