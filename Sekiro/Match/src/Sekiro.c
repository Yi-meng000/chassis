#include "Sekiro.h"

SEKIRO Sekiro = {0};
uint8_t SkillMatchType = Nine_Palace;
uint8_t WarheadFetch_Num = 1;
MatchTraceOn MatchTrace = {0};
/**
 * @brief 所有相关的初始化 在main.c中运行
 *
 */
void Program_Init(void)
{
    BEEP_Start();
    CAN_InitSendQueue();
#if USE_ZMDR
    ZdriveInit();
#endif
#if USE_VESC
    VescInit();
#endif
#if USE_DJ
    DJMotorInit();
#endif
#if USE_UNITREE
    my_Unitree_Init();
    USART_RxDMA_DoubleBuffer_Init(&huart3, (uint32_t *)(GO_rx_buf[0][0]),
                                  (uint32_t *)(GO_rx_buf[0][1]), 32);
#endif
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    USART_RxDMA_DoubleBuffer_Init(&huart1, (uint32_t *)(Radar_RxBuff[0]),
                                  (uint32_t *)(Radar_RxBuff[1]), LASER_RXBUFF);
    HAL_UART_Receive_DMA(&huart2, &rx_temp2, 1);
    HAL_UART_Receive_DMA(&huart3, &rx_temp3, 1);
    HAL_UART_Receive_IT(&huart4, &rx_temp4, 1);
    HAL_UART_Receive_DMA(&huart6, &rx_temp6, 1);
    HAL_UART_Receive_IT(&huart9, rx_temp9, QRCODE_RXLEN);
    Chassis_Init(&Chassis);
    Sekiro_Init(&Sekiro);
    chassisRunInit(&Chassis);
    TrajctoryParam_Init();
    Filter_Init(&Radar_Filter);
    VOFA_InitTxMsg(&VofaTxPack);
}
/**
 * @brief 比赛整场包括路径的默认初始化、方块的默认初始设置、取哪几个武器头
 *        比赛场上的梅花林高度
 *
 * @param sekiro
 */
void Sekiro_Init(SEKIRO *sekiro)
{
    sekiro->side = RED; // 适应备馆场地 默认为红场
    sekiro->current_zone = ZONE1;
    sekiro->offpath.path[0] = 3; // 默认从中间进入
    sekiro->offpath.path[1] = 6;
    sekiro->offpath.path[2] = 9;
    sekiro->offpath.path[3] = 12;
    // sekiro->offpath.path[4] = 11;
    sekiro->offpath.path[4] = 13;
    sekiro->offpath.rear = 4;
    sekiro->KFS_front.type = R2_KFS;

    Actparam.warhead_num[0] = 2;
    Actparam.warhead_num[1] = 3;
    side_traj = &sekiro->side;
    Sekiro.offpath.side_pick = 1;
    sekiro->offpath.pick_carPath[0] = 9;            // 到达格子9时触发侧取
    sekiro->offpath.KFS_Pos[0] = 8;                 // 侧取目标是格子8
    sekiro->Map_status.field[2].field_KFS = R2_KFS; // 格子3: R2块
    sekiro->Map_status.field[5].field_KFS = R2_KFS; // 格子6: R2块
    sekiro->Map_status.field[7].field_KFS = R2_KFS; // 格子8: R2块（侧取目标）
    sekiro->R1PlaceKFS = 1;                         // TODO
    sekiro->Docked = 0;
    // cm
    sekiro->Map_status.field[0].height = 40;
    sekiro->Map_status.field[1].height = 20;
    sekiro->Map_status.field[2].height = 40;

    sekiro->Map_status.field[3].height = 20;
    sekiro->Map_status.field[4].height = 40;
    sekiro->Map_status.field[5].height = 60;

    sekiro->Map_status.field[6].height = 40;
    sekiro->Map_status.field[7].height = 60;
    sekiro->Map_status.field[8].height = 40;

    sekiro->Map_status.field[9].height = 20;
    sekiro->Map_status.field[10].height = 40;
    sekiro->Map_status.field[11].height = 20;
    sekiro->Map_status.field[12].height = 0;
    sekiro->Map_status.field[12].field_KFS = EMPTY;
    MFCrossPos_Init(sekiro);
    KFSDiscardDicide(sekiro);
}
// 对本车所在的区域基于坐标进行判断 同时在二区时所在格子做判断
void MatchZone_Judge(CHASSIS *chassis, SEKIRO *sekiro)
{
    int16_t rx = chassis->ChassisPosReal.x;
    int16_t ry = chassis->ChassisPosReal.y;
    // 适应队里场地的数据
    if (rx <= 2450)
    {
        sekiro->current_zone = ZONE1;
        sekiro->zone2_field = 0;
    }

    else if (rx > 2300 && rx <= 7250)
    {
        sekiro->current_zone = ZONE2;

        if(rx <= 2450)
             sekiro->zone2_field = ZONE2_EXTRANCE;
        else
        {
            uint8_t tmp_x = (rx - 2450) / MEIHUA_FIELD_SIZE;
            uint8_t tmp_y = (abs(ry)) / MEIHUA_FIELD_SIZE;
            if (tmp_x >= 4)
                    tmp_x = 3;
            sekiro->zone2_field = (MEIHUA)(tmp_x * 3 + (3 - tmp_y));
        }	
    }
    else if (rx >= 7300 && rx < 9700)
    {
        sekiro->current_zone = ZONE2;
        sekiro->zone2_field = ZONE2_EXIT;
    }
    else
    {
        sekiro->current_zone = ZONE3;
        sekiro->zone2_field = 0;
    }
}
void MFCrossPos_Set(Bpoint *point, s16 tx, s16 ty, float angle)
{
    point->x = tx;
    point->y = ty * (Sekiro.side == BLUE ? 1 : -1);
    point->t = angle;
}
void MFCrossPos_Init(SEKIRO *sekiro)
{
    MFCrossPos_Set(&sekiro->MF_pos[0].forwrd_pos, 3560, 2830, 0);
    MFCrossPos_Set(&sekiro->MF_pos[0].right_pos, 3400, 2600, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[1].forwrd_pos, 3560, 1630, 0);
    MFCrossPos_Set(&sekiro->MF_pos[1].left_pos, 3400, 1860, 90.f);
    MFCrossPos_Set(&sekiro->MF_pos[1].right_pos, 3400, 1400, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[2].forwrd_pos, 3560, 430, 0);
    MFCrossPos_Set(&sekiro->MF_pos[2].left_pos, 3400, 660, 90.f);

    MFCrossPos_Set(&sekiro->MF_pos[3].forwrd_pos, 4760, 2830, 0);
    MFCrossPos_Set(&sekiro->MF_pos[3].right_pos, 4600, 2600, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[4].forwrd_pos, 4760, 1630, 0);
    MFCrossPos_Set(&sekiro->MF_pos[4].left_pos, 4600, 1860, 90.f);
    MFCrossPos_Set(&sekiro->MF_pos[4].right_pos, 4600, 1400, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[5].forwrd_pos, 4760, 430, 0);
    MFCrossPos_Set(&sekiro->MF_pos[5].left_pos, 4600, 660, 90.f);

    MFCrossPos_Set(&sekiro->MF_pos[6].forwrd_pos, 5960, 2830, 0);
    MFCrossPos_Set(&sekiro->MF_pos[6].right_pos, 5800, 2600, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[7].forwrd_pos, 5960, 1630, 0);
    MFCrossPos_Set(&sekiro->MF_pos[7].left_pos, 5800, 1860, 90.f);
    MFCrossPos_Set(&sekiro->MF_pos[7].right_pos, 5800, 1400, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[8].forwrd_pos, 5960, 430, 0);
    MFCrossPos_Set(&sekiro->MF_pos[8].left_pos, 5800, 660, 90.f);

    MFCrossPos_Set(&sekiro->MF_pos[9].forwrd_pos, 7160, 2830, 0);
    MFCrossPos_Set(&sekiro->MF_pos[9].right_pos, 7000, 2600, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[10].forwrd_pos, 7160, 1630, 0);
    MFCrossPos_Set(&sekiro->MF_pos[10].left_pos, 7000, 1860, 90.f);
    MFCrossPos_Set(&sekiro->MF_pos[10].right_pos, 7000, 1400, -90.f);

    MFCrossPos_Set(&sekiro->MF_pos[11].forwrd_pos, 7160, 430, 0);
    MFCrossPos_Set(&sekiro->MF_pos[11].left_pos, 7000, 660, 90.f);

    MFCrossPos_Set(&sekiro->Zone2Extrance_Pos[0], 2360, 2830, 0);
    MFCrossPos_Set(&sekiro->Zone2Extrance_Pos[1], 2360, 1630, 0);
    MFCrossPos_Set(&sekiro->Zone2Extrance_Pos[2], 2360, 430, 0);
}
// 单个单个设置每一个的KFS类型
bool MatchMap_Set(uint8_t type, SEKIRO *sekiro)
{
    if (sekiro->Map_status.rear >= 12)
        return 1;
    else
        sekiro->Map_status.field[sekiro->Map_status.rear++].field_KFS = (KFS_TYPE)type;
    return false;
}
// 获取在眼前的kfs 并旋转变换到 当前世界坐标系
void Camera_GetKFS(CameraRxMsgPackTypedef *rxPack, CHASSIS *chassis, SEKIRO *sekiro)
{
    if (Modulo2d((vector2d){rxPack->Floats[0], rxPack->Floats[1]}) < 1.6f)
    {
        if (rxPack->Bytes[0] == EMPTY && chassis->CamLockPoint)
            return;
        sekiro->KFS_front.type = (KFS_TYPE)rxPack->Bytes[0];
        float AngleRealRad = DEG2RAD(chassis->ChassisPosReal.angle);
        sekiro->KFS_front.pos.x = rxPack->Floats[0] * 1000.f;
        sekiro->KFS_front.pos.y = rxPack->Floats[1] * 1000.f;
        sekiro->KFS_front.pos.z = (rxPack->Floats[2] * 1000.f);
        sekiro->KFS_front.yaw = rxPack->Floats[3];
    }
}
// 将小电脑发送的路径录入
bool Route_Get(SEKIRO *sekiro)
{
    if (RouteMsg.Receive)
    {
        memcpy(sekiro->offpath.path, RouteRxPack.Bytes, (RouteMsg.RxDataSize) * sizeof(uint8_t));
        sekiro->offpath.rear = RouteMsg.RxDataSize;
        return true;
    }
    return false;
}
// 将12个格子的KFS状态录入 准备发出
bool Route_Load(SEKIRO *sekiro, uint8_t *map)
{
    if (sekiro->Map_status.rear == 12)
    {
        for (int i = 0; i < 12; i++)
            map[i] = sekiro->Map_status.field[i].field_KFS;
        return true;
    }
    return false;
}

void KFSDiscardDicide(SEKIRO *sekiro)
{
    // 统计路径上 R2_KFS 的数量
    u8 r2_on_path = 0;
    {
        u8 j = 0;
        while (sekiro->offpath.path[j] != 13 && sekiro->offpath.path[j] != 0)
        {
            if (sekiro->Map_status.field[sekiro->offpath.path[j] - 1].field_KFS == R2_KFS)
                r2_on_path++;
            j++;
        }
    }

    u8 r2_tmp = 2;
    r2_tmp -= sekiro->offpath.side_pick;
    if (sekiro->offpath.side_pick && 1 <= sekiro->offpath.KFS_Pos[0] && sekiro->offpath.KFS_Pos[0] <= 3)
    {
        r2_tmp += 1;
    }
    u8 i = 0;
    u8 KFS_pre = 0, KFS_last = 0;
    while (sekiro->offpath.path[i] != 13 && sekiro->offpath.path[i] != 0)
    {
        if (sekiro->Map_status.field[sekiro->offpath.path[i] - 1].field_KFS == R2_KFS)
        {
            if (!r2_tmp)
                sekiro->Map_status.field[sekiro->offpath.path[i] - 1].R2_KFSDiscard = true;
            else if (r2_tmp == 1)
            {
                if (KFS_last > 0)
                    sekiro->Map_status.field[KFS_last - 1].R2_KFSDiscard = true;
                sekiro->Map_status.field[sekiro->offpath.path[i] - 1].R2_KFSDiscard = false;
                KFS_last = sekiro->offpath.path[i];
            }
            else if (r2_tmp == 2)
            {
                if (KFS_pre > 0)
                    sekiro->Map_status.field[KFS_pre - 1].R2_KFSDiscard = true;
                sekiro->Map_status.field[sekiro->offpath.path[i] - 1].R2_KFSDiscard = false;
                KFS_pre = KFS_last;
                KFS_last = sekiro->offpath.path[i];
            }
        }
        i++;
    }
    // 侧取目标在第一列时：路径上只有1个R2_KFS则保留侧取目标，否则丢弃
    if (sekiro->offpath.side_pick && 1 <= sekiro->offpath.KFS_Pos[0] && sekiro->offpath.KFS_Pos[0] <= 3)
    {
        if (r2_on_path > 1)
            sekiro->Map_status.field[sekiro->offpath.KFS_Pos[0] - 1].R2_KFSDiscard = true;
        else
            sekiro->Map_status.field[sekiro->offpath.KFS_Pos[0] - 1].R2_KFSDiscard = false;
    }
}

bool KFSGetTraversal(SEKIRO *sekiro, ActuatorParam *act)
{
    if (sekiro->offpath.side_pick)
    {

        if (sekiro->offpath.front != 0 && sekiro->offpath.pick_carPath[0] == sekiro->offpath.path[sekiro->offpath.front - 1])
        {
            uint8_t tx = ((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) / 3);
            uint8_t ty = ((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) % 3);
            sekiro->DirforKFS = No_Dir;
            if ((tx + 1) * 3 + ty == sekiro->offpath.KFS_Pos[0] - 1)
                sekiro->DirforKFS = Forward;
            else if (tx * 3 + ty + 1 == sekiro->offpath.KFS_Pos[0] - 1)
                sekiro->DirforKFS = Right;
            else if (((tx - 1) >= 0 && ((tx - 1) * 3 + ty == sekiro->offpath.KFS_Pos[0] - 1)))
                sekiro->DirforKFS = Backward;
            else if (((ty - 1) >= 0 && (tx * 3 + ty - 1) == sekiro->offpath.KFS_Pos[0] - 1))
                sekiro->DirforKFS = Left;
            if (sekiro->DirforKFS)
                return true;
            else
                return false;
        }
        return false;
    }
    else
        return false;
}
// 对 方块位置 和 车身位置 的更新和判断
Task_State Match_PosUpdate(SEKIRO *sekiro, CameraRxMsgPackTypedef *rxPack, CHASSIS *chassis, u8 InControl)
{
    MatchZone_Judge(chassis, sekiro);
    Camera_GetKFS(rxPack, chassis, sekiro);
    if (sekiro->zone2_field == 13)
        return TASK_FINISH;
    if (sekiro->zone2_field != sekiro->offpath.path[sekiro->offpath.front])
    {
        return TASK_ERROR;
    }
    if (InControl)
        sekiro->offpath.front++;
    return TASK_FINISH;
}
// 在 二区方格内 对于底盘位姿的调整 基于已知的路径
void Match_PostureAdapt(SEKIRO *sekiro, CHASSIS *chassis)
{
    s16 rx = 0, ry = 0;
    s16 delta_x = 0, delta_y = 0;

    if (sekiro->task == TASK_INIT)
    {
        if (sekiro->offpath.front == 0)
        {
            if (!sekiro->KFS_Ready_Flag)
            {
                Actuator_KFSReady();
                sekiro->KFS_Ready_Flag = true;
            }
            testangle = 0;
            testLockPID = NormalPID;
            testThrehold = NormalPID;
            // if ((sekiro->offpath.path[sekiro->offpath.front] == 1 || sekiro->offpath.path[sekiro->offpath.front] == 3) && sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front]].field_KFS == R2_KFS)
            //     testPoint.x = 1100; // 进入格子3的X坐标，原值1250
            // else
            //     testPoint.x = 1300;
            // testPoint.y = (570 + (2 - ((sekiro->offpath.path[sekiro->offpath.front] - 1) % 3)) * 1200) * ((sekiro->side == BLUE) ? 1 : -1);
            testPoint.x = sekiro->Zone2Extrance_Pos[sekiro->offpath.path[0] - 1].x;
            testPoint.y = sekiro->Zone2Extrance_Pos[sekiro->offpath.path[0] - 1].y;
            if (sekiro->offpath.side_pick && sekiro->offpath.KFS_Pos[0] <= 3)
            {
                testPoint.x = sekiro->Zone2Extrance_Pos[sekiro->offpath.KFS_Pos[0] - 1].x;
                testPoint.y = sekiro->Zone2Extrance_Pos[sekiro->offpath.KFS_Pos[0] - 1].y;
            }
        }
        else if (sekiro->offpath.path[sekiro->offpath.front] == 13)
        {
            testLockPID = HarderPID;
            testThrehold = HarderPID;
            testangle = 0;
            testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].forwrd_pos.x;
            testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].forwrd_pos.y;
        }
        else
        {
            if (((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) / 3) == ((sekiro->offpath.path[sekiro->offpath.front] - 1) / 3))
            {
                if (sekiro->offpath.path[sekiro->offpath.front - 1] < sekiro->offpath.path[sekiro->offpath.front])
                {
                    testangle = -90.f * ((sekiro->side == BLUE) ? 1 : -1);
                    testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].right_pos.x;
                    testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].right_pos.y;
                }
                else
                {
                    testangle = 90.f * ((sekiro->side == BLUE) ? 1 : -1);
                    testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].left_pos.x;
                    testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].left_pos.y;
                }
            }
            else if (((sekiro->offpath.path[sekiro->offpath.front - 1]) % 3) == ((sekiro->offpath.path[sekiro->offpath.front]) % 3))
            {
                if (sekiro->offpath.path[sekiro->offpath.front - 1] < sekiro->offpath.path[sekiro->offpath.front])
                {
                    testangle = 0;
                    testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].forwrd_pos.x;
                    testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].forwrd_pos.y;
                }
                else
                    testangle = -180.f;
            }
            // testPoint.x = ZONE2StartX + (MEIHUA_FIELD_SIZE / 2) + ((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) / 3) * MEIHUA_FIELD_SIZE;
            // testPoint.x += (testangle == 0 ? 600 : 500);
            // testPoint.y = (530 + (2 - ((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) % 3)) * MEIHUA_FIELD_SIZE) * ((sekiro->side == BLUE) ? 1 : -1);
            // if (testangle == 90.f || testangle == -90.f)
            //     testPoint.y += 150 * (testangle == 90.f ? 1 : -1);
            testLockPID = HarderPID;
            testThrehold = HarderPID;
        }
        if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS != R2_KFS

            && fabs(chassis->ChassisPosReal.angle - testangle) < 5.f && sekiro->offpath.front != 0 && (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height < sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height || Modulo2d((vector2d){(chassis->ChassisPosReal.x - testPoint.x), chassis->ChassisPosReal.y - testPoint.y}) < 300))
        {
            Camera_GetKFS(&CameraRxPack, chassis, sekiro);
            if (sekiro->KFS_front.type != R1_KFS && sekiro->KFS_front.type != FAKE_KFS)
            {
                sekiro->task = TASK_FINISH;
                sekiro->state = ROBOT_WAITING;
                sekiro->state_pre = ROBOT_CHASSISRUN;
            }
        }
        else // TODO
        {
            if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS == R2_KFS)
            {
                if (sekiro->offpath.front == 0)
                {
                    if (sekiro->state_pre != ROBOT_GRABBING_KFS)
                    {
                        if (sekiro->offpath.side_pick && sekiro->offpath.KFS_Pos[0] <= 3)
                        {
                            if (sekiro->offpath.KFS_Pos[0] == 1 || sekiro->offpath.KFS_Pos[0] == 3)
                            {
                                Actuator_KFSPrepare(&Actparam, 3);
                            }
                            else
                                Actuator_KFSPrepare(&Actparam, 1);
                        }
                        else if (sekiro->offpath.path[sekiro->offpath.front] == 1 || sekiro->offpath.path[sekiro->offpath.front] == 3)
                        {
                            Actuator_KFSPrepare(&Actparam, 3);
                        }
                        else
                            Actuator_KFSPrepare(&Actparam, 1);
                    }
                }
                else
                {
                    uint8_t mode = sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height <
                                           sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height
                                       ? 1
                                       : 2;
                    if (Actparam.arm == ACT_BUSY)
                        return;
                    else
                        Actuator_KFSPrepare(&Actparam, mode);
                }
            }
            if (fabs(chassis->ChassisPosReal.angle - testangle) < 15.f)
            {
                rx = chassis->ChassisPosReal.x;
                ry = chassis->ChassisPosReal.y;
                delta_x = testPoint.x - rx;
                delta_y = testPoint.y - ry;
								float rest_dis = (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height <
																			sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height) ? 0.1 : 0.12;
                if(sekiro->offpath.front == 0)
                {
                    vector2d TestTrajPoint[5] = {{rx,ry},
                                            {rx - 40,ry + 0.3f * delta_y},
                                            {rx - 80,ry + 0.6f * delta_y},
                                            {rx ,testPoint.y - 0.1 * delta_y},
                                            {testPoint.x - 0.08 * delta_x ,testPoint.y}};
                    if(sekiro->automatic)
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 1.1f, Chassis.ChassisPosReal.angle, testangle, No_Brake);
                    else
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 1.1f, Chassis.ChassisPosReal.angle, testangle, Cross_Brake);
                }
                else if(testangle == 0 || testangle == 180)
                {
										
                    vector2d TestTrajPoint[5] = {{rx, ry},
                                                 {rx + 0.4f * delta_x, ry + 0.7f * delta_y},
                                                 {rx + 0.75f * delta_x, ry + 0.8f * delta_y},
                                                 {rx + 0.85f * delta_x, testPoint.y},
                                                 {testPoint.x - rest_dis * delta_x, testPoint.y}};
                    if (sekiro->automatic)
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 0.5f, Chassis.ChassisPosReal.angle, testangle, No_Brake);
                    else
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 0.5f, Chassis.ChassisPosReal.angle, testangle, Cross_Brake);
                }
                else if (testangle == 90 || testangle == -90)
                {
                    vector2d TestTrajPoint[5] = {{rx, ry},
                                                 {rx + 0.7f * delta_x, ry + 0.4f * delta_y},
                                                 {rx + 0.8f * delta_x, ry + 0.75f * delta_y},
                                                 {testPoint.x, ry + 0.85f * delta_y},
                                                 {testPoint.x, testPoint.y - rest_dis * delta_y}};
                    if (sekiro->automatic)
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 0.55f, Chassis.ChassisPosReal.angle, testangle, No_Brake);
                    else
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 0.55f, Chassis.ChassisPosReal.angle, testangle, Cross_Brake);
                }
                BezierParam_Init(Trajhandler + TestTraj, TestTraj, SinF, Spin,
                                 0.5f, 0.0f, 0.05f, 0.50f, 0.f, 0.0, 50, 6);
                chassis->IsRunningTraj = true;
                trajMarker = TestTraj;
                // Trajhandler[TestTraj].Inaroll = sekiro->automatic;
                sekiro->task = TASK_PROCESS;
                return;
            }
            else
            {
                rx = chassis->ChassisPosReal.x;
                ry = chassis->ChassisPosReal.y;
                delta_x = testPoint.x - rx;
                delta_y = testPoint.y - ry;
                float rest_dis = (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height <
																			sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height) ? 0.1 : 0.12;
                if(testangle == 0)
                {
                    vector2d TestTrajPoint[5] = {{rx, ry},
                                                 {rx - 0.35f * delta_x, ry + 0.7f * delta_y},
                                                 {rx + 0.05f * delta_x, ry + 0.8f * delta_y},
                                                 {rx + 0.65f * delta_x, testPoint.y},
                                                 {testPoint.x - 0.18f * delta_x, testPoint.y}};
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 0.4f, Chassis.ChassisPosReal.angle, testangle, LockPoint_Brake);
                }
                else if(testangle == 90.f || testangle == -90.f)
                {
                    vector2d TestTrajPoint[5] = {{rx, ry},
                                                 {rx + 0.7f * delta_x, ry - 0.35f * delta_y},
                                                 {rx + 0.8f * delta_x, ry + 0.05f * delta_y},
                                                 {testPoint.x, ry + 0.65f * delta_y},
                                                 {testPoint.x, testPoint.y - 0.18f * delta_y}};
                        TrajParam_SetPoints(TestTraj, Bezier, 4, TestTrajPoint, 0.35f, Chassis.ChassisPosReal.angle, testangle, LockPoint_Brake);
                }
                BezierParam_Init(Trajhandler + TestTraj, TestTraj, SinF, Spin,
                    0.35f, 0.0f, 0.05f, 0.50f, 0.15f, 0.05, 50, 6);
                chassis->IsRunningTraj = true;
                sekiro->task = TASK_PROCESS;
            }

        }
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->LockPoint && !chassis->IsRunningTraj && Actparam.arm == ACT_READY)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CHASSISRUN;
        }
    }
}

// 二区抓取KFS
void Match_GrabKFS(SEKIRO *sekiro, CHASSIS *chassis)
{
    static uint8_t step = 0;       // 侧取多步锁点步骤
    static uint16_t pause_cnt = 0; // 到位后稳定等待计数
    // 判断是否为高位侧取（目标比当前高 20cm 以上）
    bool is_high_side_pick = false;
    if (sekiro->offpath.side_pick > 0 && sekiro->offpath.front > 0 &&
        sekiro->offpath.pick_carPath[0] == sekiro->offpath.path[sekiro->offpath.front - 1])
    {
        uint8_t cur_h = sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height;
        uint8_t tgt_h = sekiro->Map_status.field[sekiro->offpath.KFS_Pos[0] - 1].height;
        if (tgt_h - cur_h >= 20)
            is_high_side_pick = true;
    }

    if (sekiro->task == TASK_INIT)
    {
        step = 0; // 初始化步骤
        switch (sekiro->DirforKFS)
        {
        case No_Dir:
            break;
        case Forward:
            testangle = 0;
            testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].forwrd_pos.x;
            testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].forwrd_pos.y;
            break;
        case Backward:
            testangle = 180.f;
            break;
        case Left:
            testangle = 90.f * ((sekiro->side == BLUE) ? 1 : -1);
            testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].left_pos.x;
            testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].left_pos.y;
            break;
        case Right:
            testangle = -90.f * ((sekiro->side == BLUE) ? 1 : -1);
            testPoint.x = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].right_pos.x;
            testPoint.y = sekiro->MF_pos[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].right_pos.y;
            break;
        default:
            break;
        }
        // 高位侧取：第一次锁点只转角度，坐标后退100mm防碰撞
        if (is_high_side_pick && sekiro->DirforKFS != No_Dir)
        {
            switch (sekiro->DirforKFS)
            {
            case Forward:
                testPoint.x -= 150;
                break; // 面向+X，后退-X
            case Backward:
                testPoint.x += 150;
                break; // 面向-X，后退+X
            case Left:
                testPoint.y -= 150 * ((sekiro->side == BLUE) ? 1 : -1);
                break; // 面向+Y，后退-Y
            case Right:
                testPoint.y += 150 * ((sekiro->side == BLUE) ? 1 : -1);
                break; // 面向-Y，后退+Y
            default:
                break;
            }
            step = 1; // 标记为高位侧取多步模式
        }
        testLockPID = HarderPID;
        testThrehold = HarderPID;
        chassis_LockPoint_ThresholdInit(chassis, 1, 1.6f, 0.1f, 5, 0.4f, 500);
        chassis->LockPoint = true;
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->LockPoint)
        {
            // 高位侧取：到位后等待稳定再进入下一步
            if (step >= 1 && step <= 3)
            {
                if (++pause_cnt < 200) // 等待 200 周期 ≈ 0.4秒
                    return;
                pause_cnt = 0;
            }
            if (step == 3) // 高位侧取后退完成 → 第四步：转到下一步移动方向
            {
                uint8_t next_field = sekiro->offpath.path[sekiro->offpath.front];
                if (next_field == 13) // 出口
                {
                    testangle = 0;
                    testPoint.x = ZONE2StartX + MEIHUA_FIELD_SIZE / 2 + 3 * MEIHUA_FIELD_SIZE + 600;
                }
                else
                {
                    uint8_t cur = sekiro->offpath.path[sekiro->offpath.front - 1];
                    if (((cur - 1) / 3) == ((next_field - 1) / 3)) // 同列
                    {
                        if (cur < next_field)
                        {
                            testangle = -90.f * ((sekiro->side == BLUE) ? 1 : -1);
                            testPoint = sekiro->MF_pos[cur - 1].right_pos;
                        }
                        else
                        {
                            testangle = 90.f * ((sekiro->side == BLUE) ? 1 : -1);
                            testPoint = sekiro->MF_pos[cur - 1].left_pos;
                        }
                    }
                    else // 同行
                    {
                        testangle = 0;
                        testPoint = sekiro->MF_pos[cur - 1].forwrd_pos;
                    }
                }
                step = 4;
                chassis->LockPoint = true;
                sekiro->task = TASK_PROCESS;
                return;
            }
            if (step == 4) // 高位侧取第四步完成 → 整个序列结束
            {
                step = 0;
                pause_cnt = 0;
                sekiro->DirforKFS = No_Dir;
                sekiro->state = ROBOT_WAITING;
                sekiro->state_pre = ROBOT_GRABBING_KFS;
                sekiro->offpath.side_pick--;
                if (sekiro->offpath.side_pick)
                {
                    sekiro->offpath.pick_carPath[0] = sekiro->offpath.pick_carPath[1];
                    sekiro->offpath.KFS_Pos[0] = sekiro->offpath.KFS_Pos[1];
                }
                return;
            }
            if (step == 1) // 高位侧取第一步完成 → 第二步：前进到取块位置
            {
                switch (sekiro->DirforKFS)
                {
                case Forward:
                    testPoint.x += 150;
                    break; // 前进+X
                case Backward:
                    testPoint.x -= 150;
                    break; // 前进-X
                case Left:
                    testPoint.y += 150 * ((sekiro->side == BLUE) ? 1 : -1);
                    break; // 前进+Y
                case Right:
                    testPoint.y -= 150 * ((sekiro->side == BLUE) ? 1 : -1);
                    break; // 前进-Y
                default:
                    break;
                }
                step = 2;
                chassis->LockPoint = true;
                sekiro->task = TASK_PROCESS;
                return;
            }
            // 正常取块或高位侧取第二步完成 → 执行抓取
            uint8_t mode = 0;
            if (sekiro->offpath.front == 0)
            {
                if (sekiro->offpath.path[sekiro->offpath.front] == 1 || sekiro->offpath.path[sekiro->offpath.front] == 3)
                    mode = 'T';
                else
                    mode = 'H';
            }
            else
            {
                uint8_t target_field;
                if (sekiro->offpath.side_pick > 0 &&
                    sekiro->offpath.pick_carPath[0] == sekiro->offpath.path[sekiro->offpath.front - 1])
                    target_field = sekiro->offpath.KFS_Pos[0] - 1;
                else
                    target_field = sekiro->offpath.path[sekiro->offpath.front] - 1;
                mode = sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height <
                               sekiro->Map_status.field[target_field].height
                           ? 'H'
                           : 'L';
            }
            sekiro->task = TASK_FINISH;
            // Actuator_KFSCatch(&Actparam, mode);

            if (mode == 'H')
                Actuator_KFSCatch(&Actparam, mode);
            else
                Actuator_KFSPrepare(&Actparam, 5);
        }
    }
    else if (sekiro->task == TASK_FINISH)
    {
        if (Actparam.arm == ACT_READY)
        {
            if (step == 2) // 高位侧取抓取完成 → 第三步：后退到安全位置
            {
                switch (sekiro->DirforKFS)
                {
                case Forward:
                    testPoint.x -= 150;
                    break; // 后退-X
                case Backward:
                    testPoint.x += 150;
                    break; // 后退+X
                case Left:
                    testPoint.y -= 150 * ((sekiro->side == BLUE) ? 1 : -1);
                    break; // 后退-Y
                case Right:
                    testPoint.y += 150 * ((sekiro->side == BLUE) ? 1 : -1);
                    break; // 后退+Y
                default:
                    break;
                }
                step = 3;
                chassis->LockPoint = true;
                sekiro->task = TASK_PROCESS;
                return;
            }
            // 正常取块完成 → 结束
            step = 0;
            pause_cnt = 0;
            sekiro->DirforKFS = No_Dir;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_GRABBING_KFS;
            sekiro->offpath.side_pick--;
            if (sekiro->offpath.side_pick)
            {
                sekiro->offpath.pick_carPath[0] = sekiro->offpath.pick_carPath[1];
                sekiro->offpath.KFS_Pos[0] = sekiro->offpath.KFS_Pos[1];
            }
        }
        chassis_LockPoint_ThresholdInit(chassis, 1, 1.6f, 0.1f, 5, 0.4f, 350);
    }
}

// 上台阶
void Match_Ascend(SEKIRO *sekiro, CHASSIS *chassis, bool height, uint8_t grab)
{
    if (sekiro->task == TASK_INIT)
    {

        chassis_Ascend(chassis, height, grab);
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!Chassis.climbover)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_ACCENDING;
        }
    }
}
// 下台阶
void Match_Descend(SEKIRO *sekiro, CHASSIS *chassis, bool height, uint8_t grab)
{
    if (sekiro->task == TASK_INIT)
    {
        chassis_Descend(chassis, height, grab);
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!Chassis.climbover)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_DESCENDING;
        }
    }
}
// 底盘跑到 武器头架子
void Match_Chassis2Warhead(SEKIRO *sekiro, ActuatorParam *act, CHASSIS *chassis)
{
    // 贴紧武器架是目标，看要余留多少给底盘不至于撞死
    // s16 tmp_x = 0;
    if (sekiro->task == TASK_INIT)
    {

        // tmp_x = 250 + act->warhead_num[act->warhead_docked] * 200;
        // vector2d TestTrajPoint[6] = {{0,0},
        //                             {tmp_x * 0.18,-100},
        //                             {tmp_x * 0.45,-300},
        //                             {tmp_x * 0.5,-450},
        //                             {tmp_x * 0.75, -570},
        //                             {tmp_x,-570}};
        // TrajParam_SetPoints(Zone1_ToWarehead,Bezier,5,TestTrajPoint,0.4 + act->warhead_num[act->warhead_docked] * 0.1f,0,0,LockPoint_Brake);
        // BezierParam_Init(Trajhandler + Zone1_ToWarehead,Zone1_ToWarehead,Square,Spin,
        //                 0.4f,0.1f,0.1f,0.5f,0.f,0.0,50,8);
        // 			trajMarker = Zone1_ToWarehead;

        Actuator_WarheadReady(act, sekiro->side);
        // chassis->IsRunningTraj = 1;

        // else
        // {
				if(act->warhead_num[act->warhead_docked] > 5)
						act->warhead_num[act->warhead_docked] = 5;
        testangle = -90 * (sekiro->side == BLUE ? 1 : -1);
        testPoint.x = 50 + act->warhead_num[act->warhead_docked] * 200;
        testPoint.y = -570 * (sekiro->side == BLUE ? 1 : -1);

        testLockPID = MediumHard;
        float _LockPointPIDParamL[3] = {0.0020f, 0.00015f, 0.0005f};
        float _LockPointPIDParamS[3] = {0.0020f, 0.00005f, 0.0005f};
        if (act->warhead_num[act->warhead_docked] <= 3)
            vector2fPIDInit(lockPointPID + MediumHard, _LockPointPIDParamS, PIDPOS);
        else
            vector2fPIDInit(lockPointPID + MediumHard, _LockPointPIDParamL, PIDPOS);
        testThrehold = 0;
        chassis->LockPoint = 1;
        // }
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->LockPoint && act->grab == ACT_READY)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CHASSISRUN;
        }
    }
}
// 抓取武器头
void Match_GrabWarhead(SEKIRO *sekiro, ActuatorParam *act)
{
    if (sekiro->task == TASK_INIT)
    {
        Actuator_WarheadCatch(act);
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (act->grab == ACT_READY)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state_pre = ROBOT_GRABBING_WARHEAD;
            sekiro->state = ROBOT_WAITING;
        }
    }
}
// 武器头 对接
void Match_WarheadDocking(SEKIRO *sekiro, ActuatorParam *act, CHASSIS *chassis)
{
    static u16 take_awayPause = 0;
    if (sekiro->task == TASK_INIT)
    {
        sekiro->task = TASK_PROCESS;
        testPoint.x = 450;
        testPoint.y = -570 * (sekiro->side == BLUE ? 1 : -1);
        testLockPID = HarderPID;
        testangle = -90 * (sekiro->side == BLUE ? 1 : -1);
        chassis->LockPoint = 1;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        // TODO 相机反馈 二维码 对接完成信号
        //				if(chassis->LockPoint)
        //					sekiro->Docked = 0;
        if (sekiro->Docked && !chassis->LockPoint)
        {
            if (take_awayPause++ > 1000)
            {
                sekiro->task = TASK_FINISH;
                sekiro->state_pre = ROBOT_DOCKING_WARHEAD;
                sekiro->state = ROBOT_WAITING;
                act->warhead_docked++;
                take_awayPause = 0;
                sekiro->Docked = 0;
            }
            if (act->warhead_docked < WarheadFetch_Num)
                Actuator_WarheadRelease(0);
            else
                Actuator_WarheadRelease(1);
        }
    }
}
// 队内场地专供 让位给R1 先出 1区
void Match_ChassisToTheEdge(SEKIRO *sekiro, CHASSIS *chassis)
{
    static uint16_t waitingR1_pause = 0;
    if (sekiro->task == TASK_INIT)
    {
        if(sekiro->Zone2Restart)
        {
            Actuator_KFSPrepare(&Actparam, 3);
            sekiro->Zone2Restart = 0;
        }
        if (sekiro->LeaveZone1 && Actparam.arm == ACT_READY)
        {
            if (sekiro->offpath.side_pick && sekiro->offpath.KFS_Pos[0] <= 3)
            {
                // testPoint.x = sekiro->Zone2Extrance_Pos[sekiro->offpath.KFS_Pos[0] - 1].x + 50;
                // testPoint.y = sekiro->Zone2Extrance_Pos[sekiro->offpath.KFS_Pos[0] - 1].y;
                // testangle = 0;
                // testLockPID = NormalPID;
                // float _LockPointPIDParam[3] = {0.0011, 0.00035, 0.00001f};
                // vector2fPIDInit(lockPointPID + NormalPID, _LockPointPIDParam, PIDPOS);
                trajMarker = Zone1_ToWarehead + sekiro->offpath.KFS_Pos[0];
                
                Trajectory_OffPointSet(Trajhandler + trajMarker,250,0);
							  if (sekiro->side == RED)
									Trajctory_CtrlPointReverse(Trajhandler + trajMarker);
                chassis->IsRunningTraj = true;
            }
            else
            {
                // testPoint.x = sekiro->Zone2Extrance_Pos[sekiro->offpath.path[0] - 1].x;
                // testPoint.y = sekiro->Zone2Extrance_Pos[sekiro->offpath.path[0] - 1].y;
                // testangle = 0;
                // testLockPID = NormalPID;
                // float _LockPointPIDParam[3] = {0.0012, 0.00035, 0.00001f};
                // vector2fPIDInit(lockPointPID + NormalPID, _LockPointPIDParam, PIDPOS);
                trajMarker = Zone1_ToWarehead + sekiro->offpath.path[0];
                
                Trajectory_OffPointSet(Trajhandler + trajMarker,200,0);
                if (sekiro->side == RED)
                    Trajctory_CtrlPointReverse(Trajhandler + trajMarker);
                chassis->IsRunningTraj = true;
            }

            // chassis_LockPoint_ThresholdInit(chassis, 0, 2.0f, 0.08f, 9, 0.5f, 1050);
            // testThrehold = NormalPID;
            // chassis->LockPoint = true;
            //            if(sekiro->offpath.path[0] == 3)
            //            {
            //                chassis->LockPoint = 0;
            //                trajMarker = Zone1EnterZone2_3rd;
            //                if (Sekiro.side == RED)
            //                    Trajctory_CtrlPointReverse(&Trajhandler[trajMarker]);
            //                chassis->IsRunningTraj = true;
            //            }

            sekiro->task = TASK_PROCESS;
        }
        else if (waitingR1_pause++ == 50)
        {
            Actuator_KFSPrepare(&Actparam, 3);
            sekiro->KFS_Ready_Flag = true;
        }
        else if (waitingR1_pause > 3500)
        {
            sekiro->LeaveZone1 = true;
        }
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->IsRunningTraj && Actparam.arm == ACT_READY)
        {
            sekiro->task = TASK_FINISH;
            testThrehold = 0;
            waitingR1_pause = 0;
            sekiro->LeaveZone1 = 0;
            if (sekiro->offpath.KFS_Pos[0] == 2)
                Actuator_KFSPrepare(&Actparam, 1);
            else if (sekiro->offpath.path[0] == 2 && sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].field_KFS == R2_KFS)
                Actuator_KFSPrepare(&Actparam, 1);
        }
    }
    else if (sekiro->task == TASK_FINISH)
    {
        if (Actparam.arm == ACT_READY)
        {
            sekiro->state_pre = ROBOT_CHASSISRUN;
            sekiro->state = ROBOT_WAITING;
            chassis_LockPoint_ThresholdInit(chassis, 0, 2.f, 0.08f, 6, 0.5f, 800);
        }
    }
}
// 对接结束 到 2区 门口
void Match_Chassis2Zone2Entrance(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        testPoint.x = ZONE2StartX; // 场地误差
        if (sekiro->offpath.path[0] == 1 || sekiro->offpath.path[0] == 3)
            testPoint.x = ZONE2StartX - 50;
        testPoint.y = (500 + (sekiro->offpath.path[0] - 1) * MEIHUA_FIELD_SIZE) * (sekiro->side == BLUE ? 1 : -1);
        testangle = 0;
        testLockPID = NormalPID;
        chassis->LockPoint = 1;
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->LockPoint)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state_pre = ROBOT_CHASSISRUN;
            sekiro->state = ROBOT_WAITING;
        }
    }
}

// 放置九宫格 中层
void Match_KFSPutMiddle(SEKIRO *sekiro, CHASSIS *chassis, ActuatorParam *act)
{
    if (sekiro->task == TASK_INIT)
    {
        sekiro->task = TASK_PROCESS;
        Actuator_KFSPlace(act, 'M');
        RobotRxmsg.receive_effect = false;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (act->arm == ACT_READY)
        {
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_PLACING_KFS;
            RobotRxmsg.receive_effect = true;
            sekiro->R1PlaceKFS = 2;
            // chassis_Upstand(chassis, CHASSIS_HEIGHT_ZERO);
        }
    }
}
// 放置九宫格 顶层
void Match_KFSPutTop(SEKIRO *sekiro, ActuatorParam *act)
{
    if (sekiro->task == TASK_INIT)
    {
        sekiro->task = TASK_PROCESS;
        Actuator_KFSPlace(act, 'T');
        RobotRxmsg.receive_effect = false;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (act->arm == ACT_READY)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_PLACING_KFS;
            sekiro->R1PlaceKFS = 0;
            RobotRxmsg.receive_effect = true;
        }
    }
}
// 以 相机 为 准 纠正 雷达的坐标
bool Camera_Relocation(SEKIRO *sekiro, SENSORUSART_MSG *sense_msg, CameraRxMsgPackTypedef *rxPack, CHASSIS *chassis)
{

    float AngleRad = DEG2RAD(chassis->ChassisPosReal.angle);
    if (rxPack->Bytes[0] == EMPTY)
        return false;
    // 350为车的大小
    // 格子的中心坐标
    //    int16_t block_x = ZONE2StartX + 317 + ((sekiro->offpath.path[sekiro->offpath.front]-  1 ) / 3) * 1200 + MEIHUA_FIELD_SIZE / 2;
    //    int16_t block_y = ZONE2StartY  + (-317 + (2 - ((sekiro->offpath.path[sekiro->offpath.front]-  1 ) % 3)) * 1200 + MEIHUA_FIELD_SIZE / 2)
    //                * (sekiro->side == BLUE ? 1 : -1);

    float rx = (sekiro->KFS_front.pos.x * cosf(AngleRad) + sekiro->KFS_front.pos.y * sinf(AngleRad)) * 1000.f;
    float ry = (sekiro->KFS_front.pos.y * cosf(AngleRad) - sekiro->KFS_front.pos.y * sinf(AngleRad)) * 1000.f;

    // sense_msg->offset_x =   block_x - (int16_t)(rx ) - (Sensor_RxPack.Laser_RxPack.floats[0] * 1000.f);
    // sense_msg->offset_y =  	block_y - (int16_t)(ry ) - (Sensor_RxPack.Laser_RxPack.floats[1] * 1000.f);
    if (testangle == 0 || testangle == 180.f)
        sense_msg->offset_x -= KFS_EdgeLen / 2 * (testangle == 0 ? 1 : -1);
    else if (testangle == 90.f || testangle == -90.f)
        sense_msg->offset_y -= KFS_EdgeLen / 2 * (testangle == -90.f ? 1 : -1);
    return true;
}

void Match_High400mmGrabKFS(SEKIRO *sekiro, CHASSIS *chassis, ActuatorParam *act)
{
    if (sekiro->task == TASK_INIT)
    {
        if (sekiro->offpath.KFS_Pos[0] != 0)
        {
            if (sekiro->Map_status.field[sekiro->offpath.KFS_Pos[0] - 1].R2_KFSDiscard)
            {
                if (sekiro->offpath.KFS_Pos[0] == 1 || sekiro->offpath.KFS_Pos[0] == 3)
                {
                    Actuator_KFSCatch(act, 'B');
                    chassis_Upstand(chassis, CHASSIS_HEIGHT_200mm);
                }
                else
                    Actuator_KFSCatch(act, 'A');
            }
            else
            {
                if (sekiro->offpath.KFS_Pos[0] == 1 || sekiro->offpath.KFS_Pos[0] == 3)
                    chassis_Upstand(chassis, CHASSIS_HEIGHT_200mm);
                else
                    Actuator_KFSCatch(act, 'K');
            }
        }

        sekiro->task = TASK_PROCESS;
        act->arm = ACT_BUSY;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->StandUp && act->arm == ACT_READY)
        {
            chassis_Upstand(chassis, CHASSIS_HEIGHT_ZERO);
            sekiro->task = TASK_FINISH;
        }
    }
    else if (sekiro->task == TASK_FINISH)
    {
        if (!chassis->StandUp)
        {
            sekiro->task = TASK_INIT;
            sekiro->state_pre = ROBOT_GRABBING_KFS;
            sekiro->state = ROBOT_WAITING;
            sekiro->offpath.side_pick--;
            if (sekiro->offpath.side_pick)
            {
                sekiro->offpath.pick_carPath[0] = sekiro->offpath.pick_carPath[1];
                sekiro->offpath.KFS_Pos[0] = sekiro->offpath.KFS_Pos[1];
            }
        }
    }
}
void Match_CameraChassisAdapt(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        if (sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front] - 1].height >
            sekiro->Map_status.field[sekiro->offpath.path[sekiro->offpath.front - 1] - 1].height)
            CamDepth = 920;
        else
            CamDepth = 1050;
        chassis->CamLockPoint = true;
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->CamLockPoint)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CAMERA_POSADAPT;
        }
    }
}
// 爬坡
void Match_ClimbSlope(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        chassis->slopeover = true;
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->slopeover)
        {
            sekiro->Slope_Climb = true;
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CLIMBSLOPE;
        }
    }
}
// 从梅林出来到斜坡入口
void Match_ChassisToZone3Entrance(SEKIRO *sekiro, CHASSIS *chassis)
{
    static s16 delta_x = 0, delta_y = 0;
    static s16 rx = 0, ry = 0;
    if (sekiro->task == TASK_INIT)
    {
        rx = chassis->ChassisPosReal.x;
        ry = chassis->ChassisPosReal.y;
        delta_x = ZONE3_SLOPE_ENTRANCE_X - rx;
        delta_y = ZONE3_SLOPE_ENTRANCE_Y - ry;
        vector2d BeforeSlope[6] = {
            {rx, ry},
            {rx + 0.4f * delta_x, ry + 0.25f * delta_y},
            {rx + 0.4f * delta_x, ry + 0.6f * delta_y},  // P2 x 不能等于 P3
            {rx + 0.5f * delta_x, ry + 0.88f * delta_y}, // P3 x 继续推进
            {ZONE3_SLOPE_ENTRANCE_X - 0.5f * delta_x, ZONE3_SLOPE_ENTRANCE_Y},
            {ZONE3_SLOPE_ENTRANCE_X, ZONE3_SLOPE_ENTRANCE_Y}};
        if (abs(chassis->ChassisPosReal.y) < 2210)
        {
            TrajParam_SetPoints(Zone3_BeforeSlope, Bezier, 5, BeforeSlope, 1.8f, 0, 0, No_Brake);
            BezierParam_Init(Trajhandler + Zone3_BeforeSlope, Zone3_BeforeSlope, SinF, Spin,
                             1.7f, 0.1f, 0.05f, 0.5f, 0, 0, 0.8f, 80);
        }
        else
        {
            TrajParam_SetPoints(Zone3_BeforeSlope, Bezier, 5, BeforeSlope, 1.3f, 0, 0, Cross_Brake);
            BezierParam_Init(Trajhandler + Zone3_BeforeSlope, Zone3_BeforeSlope, SinF, Spin,
                             1.2f, 0.1f, 0.05f, 0.5f, 0, 0, 0.8f, 80);
        }
        trajMarker = Zone3_BeforeSlope;
        chassis->IsRunningTraj = true;
        // testangle = 0.f;
        // // testPoint.x = ZONE3_SLOPE_ENTRANCE_X;
        // // testPoint.y = ZONE3_SLOPE_ENTRANCE_Y * (sekiro->side == BLUE ? 1 : -1);
        // testLockPID = NormalPID;
        // testThrehold = 0;
        // chassis_LockPoint_ThresholdInit(chassis, 0, 2.5f, 0.08f, 30, 0.5f, 1000);
        // chassis->LockPoint = 1;
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->IsRunningTraj)
        {
            // chassis_LockPoint_ThresholdInit(chassis, 0, 2.5f, 0.08f, 6, 0.5f, 1000);
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CHASSISRUN;
            sekiro->Slope_Climb = 1;
        }
    }
}
// 爬坡后和r1汇合 准备上R1
void Match_ChassisToSquare9(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        chassis->IsRunningTraj = true;
        if (!sekiro->Zone3Restart)
            trajMarker = Zone3_SlopeToEnd;
        else
            trajMarker = Zone3_AfterSlope;
        if (sekiro->side == RED)
            Trajctory_CtrlPointReverse(&Trajhandler[trajMarker]);
        testLockPID = HarderPID;
        testThrehold = HarderPID;
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->IsRunningTraj)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CHASSISRUN;
            sekiro->SlopeAfter = true;
        }
    }
}
// 与R1合体
void Match_UptoR1(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        if (Camera_SearchR1(sekiro, chassis))
        {
            chassis->ClimbUp2R1 = 0;
            chassis_Up2R1(chassis);
            sekiro->task = TASK_PROCESS; // TODO
        }
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (chassis->ClimbUp2R1)
        {
            sekiro->task = TASK_FINISH;
            Actuator_KFSPlace(&Actparam, 'P');
        }
    }
    else if (sekiro->task == TASK_FINISH)
    {
        if (Actparam.arm == ACT_READY)
        {
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_UPTOR1;
            sekiro->task = TASK_INIT;
        }
    }
}
// 获取R1的方块
void Match_GrabR1KFS(SEKIRO *sekiro, ActuatorParam *act)
{
    if (sekiro->task == TASK_INIT)
    {
        Actuator_KFSGrabR1Block(act);
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (act->arm == ACT_READY)
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_GRABR1KFS;
            sekiro->R1PlaceKFS = 0;
        }
    }
}

bool Camera_SearchR1(SEKIRO *sekiro, CHASSIS *chassis)
{
    static u8 search_phase = 0, receive_cnt = 0;
    static Bpoint average_point = {0};
    static u8 lock_time = 0;
    s16 rx = 0, ry = 0;
    s16 delta_x = 0, delta_y = 0;
    s16 aim_x = 0;

    switch (search_phase)
    {
    case 0:
        RobotRxmsg.pos_receive = 0;
        receive_cnt = 0;
        search_phase++;

        break;
    case 1:
        if (RobotRxmsg.pos_receive)
        {

            // rx = chassis->ChassisPosReal.x;
            // ry = chassis->ChassisPosReal.y;

            if (fabs(RobotRxmsg.ToR1_y) < 350)
                aim_x = 10390;
            else if (RobotRxmsg.ToR1_y > 350)
                aim_x = (sekiro->side == BLUE ? 10930 : 9850);
            else if (RobotRxmsg.ToR1_y < -350)
                aim_x = (sekiro->side == BLUE ? 9850 : 10930);
            // delta_x = aim_x - rx;
            // delta_y = 600 * (sekiro->side ? 1 : -1) - ry;
            // vector2d TestTrajPoint[6] = {{rx,ry},
            //                         {rx + 0.2f * delta_x,ry + 0.2f * delta_y},
            //                         {rx + 0.4f * delta_x,ry + 0.4f * delta_y},
            //                         {rx + 0.6f *delta_x ,ry + 0.6f * delta_y},
            //                         {rx + 0.8f * delta_x,ry + 0.8f * delta_y},
            //                         {aim_x, 600 * (sekiro->side ? 1 : -1 )}};


            // TrajParam_SetPoints(TestTraj, Bezier, 5, TestTrajPoint, 0.8f, -90.f * (sekiro->side == BLUE ? 1: -1), -90.f * (sekiro->side == BLUE ? 1: -1), Cross_Brake);
            // BezierParam_Init(Trajhandler + TestTraj, TestTraj, SinF, Spin,
            //             0.7f, 0.0f, 0.2f, 0.35f, 0.f, 0.0, 40, 10);
            // trajMarker = TestTraj;
            // chassis->IsRunningTraj = true;
            testPoint.x = aim_x;
            testPoint.y = 600 * (sekiro->side == BLUE ? 1 : -1);
            testangle = -90.f * (sekiro->side == BLUE ? 1 : -1);
            chassis->LockPoint = true;
            testLockPID = HarderPID;
            testThrehold = HarderPID;
            RobotRxmsg.pos_receive = 0;
            search_phase++;
        }
        break;
    case 2:

        if (!chassis->LockPoint && RobotRxmsg.pos_receive)
        {
            if (++receive_cnt <= 5)
            {
                RobotRxmsg.pos_receive = 0;
                if (Modulo2d((vector2d){RobotRxmsg.ToR1_x, RobotRxmsg.ToR1_y}) > 1200)
                {
                    receive_cnt--;
                    return 0;
                }
                average_point.x += RobotRxmsg.ToR1_x;
                average_point.y += RobotRxmsg.ToR1_y;
                average_point.t += RobotRxmsg.ToR1Angle;
            }
            else
            {
                average_point.x /= 5;
                average_point.y /= 5;
                average_point.t /= 5;
                float rad = DEG2RAD(chassis->ChassisPosReal.angle);
                float body_x = (float)average_point.x - 600.0f;
                float body_y = (float)average_point.y;

                testPoint.x = chassis->ChassisPosReal.x + roundf(body_x * cosf(rad) - body_y * sinf(rad));
                testPoint.y = chassis->ChassisPosReal.y + roundf(body_x * sinf(rad) + body_y * cosf(rad));
                testangle = chassis->ChassisPosReal.angle;
                testLockPID = HarderPID;

                chassis->LockPoint = true;
                search_phase++;
                RobotRxmsg.pos_receive = 0;
                receive_cnt = 0;
                lock_time++;
                average_point.x = 0;
                average_point.y = 0;
                average_point.t = 0;
            }
        }
        else
            RobotRxmsg.pos_receive = 0;
        break;
    case 3:
        if (!chassis->LockPoint)
        {
            if (RobotRxmsg.pos_receive)
            {
                if (++receive_cnt <= 5)
                {
                    RobotRxmsg.pos_receive = 0;
                    average_point.x += RobotRxmsg.ToR1_x;
                    average_point.y += RobotRxmsg.ToR1_y;
                    average_point.t += RobotRxmsg.ToR1Angle;
                    return false;
                }
                else
                {
                    average_point.x /= 5;
                    average_point.y /= 5;
                    receive_cnt = 0;
                    if (abs(average_point.x - 600) < 15 && abs(average_point.y) < 8)
                    {
                        average_point.x = 0;
                        average_point.y = 0;
                        average_point.t = 0;
                        return true;
                    }
                    else
                    {
                        search_phase--;
                        average_point.x = 0;
                        average_point.y = 0;
                        average_point.t = 0;
                        if (lock_time > 1)
                        {
                            lock_time = 0;
                            return true;
                        }
                        else
                            return false;
                    }
                }
            }
        }
        else
            RobotRxmsg.pos_receive = 0;
        break;
    default:
        break;
    }
    return false;
}
void Match_ChassisSearchRoutes(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        Match_PosUpdate(sekiro, &CameraRxPack, chassis, 0);
        if (testangle != 0 && !sekiro->SearchBefore)
        {
            testangle = 0;
            testPoint.x = ZONE2StartX + (MEIHUA_FIELD_SIZE / 2) + ((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) / 3) * MEIHUA_FIELD_SIZE;
            testPoint.x += 565;
            testPoint.y = (600 + (2 - ((sekiro->offpath.path[sekiro->offpath.front - 1] - 1) % 3)) * MEIHUA_FIELD_SIZE) * ((sekiro->side == BLUE) ? 1 : -1);
            testLockPID = HarderPID;
            testThrehold = HarderPID;
            chassis->LockPoint = true;
        }
        else
        {
            sekiro->SearchBefore = true;
            // if((sekiro->zone2_field - 1) / 3 == 1 && )
        }
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->LockPoint)
        {
            if (!sekiro->SearchBefore)
            {
                sekiro->task = TASK_INIT;
                sekiro->SearchBefore = true;
            }
            // else if()
            // {

            // }
        }
    }
    else if (sekiro->task == TASK_FINISH)
    {
    }
}

void SkillMatch_GrabKFS(SEKIRO *sekiro, ActuatorParam *act, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        Actuator_KFSCatch(act, 'P');
        sekiro->task = TASK_PROCESS;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (act->arm == ACT_READY)
        {

            if (Actparam.KFS_load == 2)
            {
                testPoint.x = 10350;
                testPoint.y = 1080 * (sekiro->side == BLUE ? 1 : -1);
                testangle = -90 * (sekiro->side == BLUE ? 1 : -1);
                chassis->LockPoint = true;
                sekiro->task = TASK_FINISH;
            }
            else
            {
                sekiro->state = ROBOT_WAITING;
                sekiro->state_pre = ROBOT_GRABBING_KFS;
            }
        }
    }
    else if (sekiro->task == TASK_FINISH)
    {
        if (!chassis->LockPoint)
        {
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_GRABBING_KFS;
        }
    }
}

void SkillMatch_ChassisAdapt(SEKIRO *sekiro, CHASSIS *chassis)
{
    if (sekiro->task == TASK_INIT)
    {
        if (Actparam.KFS_load == 0)
        {
            testPoint.x = 10350;
            testPoint.y = 2800 * (sekiro->side == BLUE ? 1 : -1);
            testangle = 0;
        }
        else if (Actparam.KFS_load == 1)
        {
            testPoint.x = 10350;
            testPoint.y = 2400 * (sekiro->side == BLUE ? 1 : -1);
            testangle = 0;
        }
        testLockPID = HarderPID;
        chassis->LockPoint = true;
    }
    else if (sekiro->task == TASK_PROCESS)
    {
        if (!chassis->LockPoint)
        {
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_CHASSISRUN;
        }
    }
}
