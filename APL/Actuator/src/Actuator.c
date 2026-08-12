#include "ActcatorCom.h"
#include "chassisComm.h"
ActuatorParam Actparam;
FDCAN_RxHeaderTypeDef rxheader = {0};
uint8_t data[8] = {0};
void Actuator_Enable(ActuatorParam *act, uint8_t enable)
{

    HeaderPrepare(MASTER_ACTUATOR_ENABLE, 2, &rxheader);
    data[0] = 'M';
    data[1] = enable;
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
        act->arm = enable ? ACT_READY : ACT_DISABLE;
        act->grab = enable ? ACT_READY : ACT_DISABLE;
    }
}
void Actuator_ReceiveHandler(ActuatorParam *act, uint8_t rxdata[], FDCAN_RxHeaderTypeDef rxheader)
{
    switch (rxheader.Identifier)
    {
    case SLAVE_ACTUATOR_WARHEAD_CATCH:
        act->grab = ACT_READY;
        break;
    case SLAVE_ACTUATOR_ARM_READY:
        act->arm = ACT_READY;
        break;
    case SLAVE_ACTUATOR_WARHEAD_READY:
        act->grab = ACT_READY;
        break;
    case SLAVE_ACTUATOR_ARM_CATCH:
        act->arm = ACT_READY;
        act->KFS_load++;
        if(rxdata[0] == 'L')
            chassis_Upstand(&Chassis,CHASSIS_HEIGHT_ZERO);
        // if(rxdata[2] == 'L')
        //     act->left_occupied++;
        // else if(rxdata[2] == 'R')
        //     act->right_occupied++;
        break;
    case SLAVE_ACTUATOR_ARM_GRAB:
        act->arm = ACT_READY;
        act->KFS_load++;
        // if(rxdata[2] == 'L')
        //     act->left_occupied++;
        // else if(rxdata[2] == 'R')
        //     act->right_occupied++;
        break;
    case SLAVE_ACTUATOR_ARM_PREPARE:
        act->arm = ACT_READY;
        break;
    case SLAVE_ACTUATOR_ARM_PLACE:
        act->KFS_load--;
        act->arm = ACT_READY;
        // if(rxdata[2] == 'L')
        //     act->left_occupied = 0;
        // else if(rxdata[2] == 'R')
        //     act->right_occupied = 0;
        break;
    case SLAVE_ACTUATOR_ERROR:
        if (rxdata[0] == 'C')
            act->grab = ACT_ERROR;
        else if (rxdata[0] == 'A')
            act->arm = ACT_ERROR;
        break;
    default:
        break;
    }
}
void Actuator_Reset(ActuatorParam *act)
{
    HeaderPrepare(MASTER_ACTUATOR_RESET, 2, &rxheader);
    data[0] = 'R';
    data[1] = 'S';
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
        act->arm = ACT_DISABLE;
    }
}

void Actuator_KFSCatch(ActuatorParam *act, uint8_t mode)
{
    HeaderPrepare(MASTER_ACTUATOR_ARM_CATCH, 2, &rxheader);
    data[0] = 'B';
    data[1] = mode;
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
        act->arm = ACT_BUSY;
    }
}

void Actuator_KFSPlace(ActuatorParam *act, uint8_t cell)
{
    HeaderPrepare(MASTER_ACTUATOR_ARM_PLACE, 2, &rxheader);
    data[0] = 'B';
    data[1] = cell;
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
        act->arm = ACT_BUSY;
    }
}

void Actuator_KFSGrabR1Block(ActuatorParam *act)
{
    HeaderPrepare(MASTER_ACTUATOR_ARM_GRAB, 2, &rxheader);
    data[0] = 'B';
    data[1] = 'G';
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
        act->arm = ACT_BUSY;
    }
}
// void Actuator_KFSLoad(ActuatorParam *act)
// {
//     HeaderPrepare(MASTER_ACTUATOR_ARM_LOAD,2,&rxheader);
//     data[0] = 'B';
//     data[1] = 'L';
//     if(CAN_Queue_IfFull(&CAN2_Txqueue))
//         Can2FullFlag++;
//     else
//     {
//         CAN_Enqueue(&CAN2_Txqueue,rxheader,data);
//         act->arm = ACT_BUSY;
//     }
// }
void Actuator_KFSReady(void)
{
    HeaderPrepare(MASTER_ACTUATOR_ARM_READY, 2, &rxheader);
    data[0] = 'B';
    data[1] = 'Z';
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        Actparam.arm = ACT_BUSY;
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}

void Actuator_WarheadCatch(ActuatorParam *act)
{
    HeaderPrepare(MASTER_ACTUATOR_WARHEAD_CATCH, 3, &rxheader);
    data[0] = 'C';
    data[1] = 'D';
    if (act->warhead_num[act->warhead_docked] == 2 || act->warhead_num[act->warhead_docked] == 3)
        data[2] = 'P';
    else if (act->warhead_num[act->warhead_docked] == 1 || act->warhead_num[act->warhead_docked] == 4)
        data[2] = 'Q';
    else
        data[2] = 'S';
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
        act->grab = ACT_BUSY;
    }
}
// void Actuator_WarheadMove(ActuatorParam *act,bool side)
// {543.05800795  271.529003975
//     HeaderPrepare(MASTER_ACTUATOR_WARHEAD_MOVE,2,&rxheader);
//     data[0] = 'C';
//     data[1] = (side == 1 ? 'B' : 'R');
//     if(CAN_Queue_IfFull(&CAN2_Txqueue))
//         Can2FullFlag++;
//     else
//     {
//         CAN_Enqueue(&CAN2_Txqueue,rxheader,data);
//         act->arm = ACT_BUSY;
//     }

// }
void Actuator_KFSPrepare(ActuatorParam *act, u8 ready_mode)
{
    HeaderPrepare(MASTER_ACTUATOR_ARM_PREPARE, 2, &rxheader);
    data[0] = 'R';
    switch (ready_mode)
    {
    case 1:
        data[1] = 'H';
        break;
    case 2:
        data[1] = 'L';
        break;
    case 3:
        data[1] = 'T';
        break;
    case 4:
        data[1] = 'P';
        break;
    case 5:
        data[1] = 'M';
        break;
    default:
        break;
    }
    act->arm = ACT_BUSY;
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}
void Actuator_WarheadReady(ActuatorParam *act, bool side)
{
    HeaderPrepare(MASTER_ACTUATOR_WARHEAD_READY, 2, &rxheader);
    data[0] = 'C';
    data[1] = 'W';
    act->grab = ACT_BUSY;
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}
void Actuator_WarheadRelease(bool finish)
{
    HeaderPrepare(MASTER_ACTUATOR_WARHEAD_RELEASE, 3, &rxheader);
    data[0] = 'C';
    data[1] = 'R';
    data[2] = finish ? 'Y' : 'N';
    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}

void Actuator_DMSetZero(void)
{
    HeaderPrepare(MASTER_ACTUATOR_DM_SET_ZERO, 2, &rxheader);
    data[0] = 'D';
    data[1] = 'Z';

    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}

void Actuator_UpClaw(u8 dis)
{
    HeaderPrepare(MASTER_ACTUATOR_UPCLAW, 2, &rxheader);
		data[0] = 'U';
    data[1] = dis;

    if (CAN_Queue_IfFull(&CAN2_Txqueue))
        Can2FullFlag++;
    else
    {
        CAN_Enqueue(&CAN2_Txqueue, rxheader, data);
    }
}
