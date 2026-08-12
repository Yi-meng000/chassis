# Sekiro

> R2 主控 · WHUROBOCON · 武汉大学 RoboCon

Sekiro 是 R2 的主控制器仓库，运行于 STM32H723ZET + FreeRTOS v10.3.1。负责全自动比赛的状态调度、底盘控制、多传感器融合、执行器驱动以及 R1-R2 双机协作。

---

## 目录

- [架构](#架构)
- [任务系统](#任务系统)
- [状态机](#状态机)
- [底盘控制](#底盘控制)
- [通讯协议](#通讯协议)
- [电机驱动](#电机驱动)
- [调试](#调试)
- [编译](#编译)
- [已知问题](#已知问题)

---

## 架构

```
Sekiro/              比赛逻辑层 — 状态机调度、区域执行、R1-R2 通讯
APL/                 应用层 — 底盘/电机/传感器/执行器
BBL/                 行为层 — FreeRTOS 任务、调试控制器、轨迹生成
FML/                 基础层 — CAN 队列、PID、数学工具、IRQ 路由
Core/                CubeMX 生成的 HAL 初始化（勿手动编辑）
```

**启动流程**：`main()` → HAL 初始化 → `Program_Init()` (CAN 队列、电机、UART DMA、底盘、状态机、PID、轨迹、滤波器) → FreeRTOS 调度器启动。

---

## 任务系统

| 任务 | 周期 | 优先级 | 职责 |
|------|------|--------|------|
| `SekiroTask` | 2ms | 最高 | 比赛状态机，决策"做什么" |
| `TaskTrajctory` | 2ms | 最高 | 贝塞尔轨迹跟踪 |
| `TaskChassis` | 2ms | High7 | 底盘 PID 闭环，执行"怎么做" |
| `TaskUart` | 10ms | High | 蓝牙调试收发 |
| `Testtask` | 10ms | Low | 位置更新、转向角查询 |
| `StartDefaultTask` | 连续 | 最低 | LED 流水灯 |

**控制链路**：SekiroTask 设置目标位置/角度 + 触发标志位（`LockPoint` / `CamLockPoint` / `slopeover`）→ TaskChassis 每 2ms 执行 PID 闭环 → `sendCarVel()` 经 FDCAN2 发送到底盘从板。

决策层与控制层通过 `CHASSIS` 结构体中的 `volatile bool` 标志位异步解耦。SekiroTask 设置标志位后在 `TASK_PROCESS` 中轮询等待；TaskChassis 检测到标志位后启动 PID，收敛后清零标志位。

---

## 状态机

### 层级结构

```
Sekiro_ShinobiExecution()              全自动比赛
  ├─ MatchPhase=0: Match_Init          等待底盘+执行器使能
  ├─ MatchPhase=1: MartialClub         Zone1: 取弹头 → 对接 → 循环
  ├─ MatchPhase=2: MFCross             Zone2: 锁点导航 → 上下台阶 → 侧取KFS
  └─ MatchPhase=3: Tictactoe           Zone3: 爬坡 → 上R1 → 放块 → 取R1块

Sekiro_ShadowDiesTwice()               技能赛（可通过 SKILLMATCH_MODE 配置）
```

### 动作函数模板

所有比赛动作遵循统一的三阶段模式：

```c
void Match_动作名(SEKIRO *sekiro, CHASSIS *chassis)
{
    if(sekiro->task == TASK_INIT)
    {
        // 设置目标位置/角度，选择 PID 精度，触发动作
        testPoint.x = 目标X;  testPoint.y = 目标Y;  testangle = 目标角度;
        testLockPID = NormalPID;  testThrehold = NormalPID;
        chassis->LockPoint = true;
        sekiro->task = TASK_PROCESS;
    }
    else if(sekiro->task == TASK_PROCESS)
    {
        if(!chassis->LockPoint)  // PID 收敛
        {
            sekiro->task = TASK_FINISH;
            sekiro->state = ROBOT_WAITING;
            sekiro->state_pre = ROBOT_刚完成的动作;  // 告诉上层状态机
        }
    }
}
```

### Zone2 核心流程 (`Sekiro_MFCross`)

```
WAITING ──→ CHASSISRUN ──→ ACCENDING / DESCENDING
  ↑              │
  │              └──→ GRABBING_KFS (侧取)
  └────────────────────────┘
```

- `Match_PostureAdapt`：PID 锁点到下一格子中心，根据路径方向计算目标位姿
- `Match_GrabKFS`：相机锁点 → 根据高度差选择抓取模式（T/H/L）
- `Match_Ascend` / `Match_Descend`：发送升降指令，等待 `climbover` 回传
- `KFSGetTraversal`：判断是否有相邻格子的侧取机会

### Zone3 核心流程 (`Sekiro_Tictactoe`)

```
WAITING → CLIMBSLOPE → CHASSISRUN → UPTOR1 → PLACING_KFS → GRABR1KFS
```

- `Match_ClimbSlope`：触发 5 状态爬坡机（由 TaskChassis 执行）
- `Match_KFSPutMiddle`：升降底盘到 400mm → 放块 → 降回
- `Match_KFSPutTop`：直接放顶层

---

## 底盘控制

### 运动学

四轮舵轮（swerve drive），轮组安装角 FL=135° / FR=45° / BR=-45° / BL=-135°。

```
Vxᵢ = Vx + ω·R·cos(φᵢ)      R = 0.338m
Vyᵢ = Vy + ω·R·sin(φᵢ)      轮径 = 86.5mm
```

`wheelTurnMin()` 确保转向角差 < 90°，必要时反转驱动方向。

### PID 控制器

| 控制器 | 类型 | 用途 | P / I / D |
|--------|------|------|-----------|
| `lockPointPID[NormalPID]` | 2D | 普通锁点 | 0.0011 / 0.00025 / 0.00001 |
| `lockPointPID[HarderPID]` | 2D | 精密锁点 | 0.0015 / 0.00035 / 0.000025 |
| `lockPointPID[MediumHard]` | 2D | 中等精度 | 0.0012 / 0.00035 / 0.000015 |
| `anglePID` | 标量 | 航向保持 | 1.70 / 0.6 / 0.0025 |
| `trajPID` | 2D | 轨迹跟踪 | 0.0011 / 0.00035 / 0.00001 |

收敛条件：位置误差 < 6mm 且角度误差 < 0.5°，持续 6 个周期（12ms）。

### 轨迹跟踪

- **路径定义**：贝塞尔曲线（最高 8 阶），6 控制点
- **弧长参数化**：50 段查找表 + Newton-Raphson 精化
- **速度剖面**：梯形（Square）或正弦 S 曲线（SinF）
- **跟踪修正**：2D 向量 PID + 自适应时间推进
- **到达制动**：`LockPoint_Brake`（PID 锁到终点）/ `Cross_Brake`（交叉锁轮）/ `No_Brake`

### 爬坡状态机 (`chassis_SlopeClimb`)

| 状态 | 条件 | 速度 | 说明 |
|------|------|------|------|
| SlopeBottom | pitch < 1.5° | 2.0 m/s | 平地前进 |
| SlopeBottomEdge | pitch → 15° | 2.0 m/s | 接近坡底 |
| SlopeOn | pitch ≈ 15° | 2.2 m/s | 坡上行驶 |
| SlopeTopEdge | pitch → 0° | 减速 | 接近坡顶 |
| SlopeTop | pitch < 0.8° 持续 10 周期 | — | 启动 Zone3 轨迹 |

---

## 通讯协议

### CAN 总线

ID 编码：`0xAABBCCDD`（AA=发送方, BB=通讯路, CC=接收方, DD=操作码）

设备代号：`0x01` 主控 · `0x02` 底盘 · `0x05` 执行器

#### 主控 → 底盘

| CAN ID | 功能 | 数据 | 说明 |
|--------|------|------|------|
| `0x01020201` | 使能 | `M` + enable | 使能/失能 |
| `0x01020203` | 速度 | vx(s16) + vy(s16) + vw(s16) + mode(s16) | ×1000 / ×100 |
| `0x01020204` | 上升 | `A` + grab + height | 翻越台阶 |
| `0x01020205` | 下降 | `D` + grab + height | 下台阶 |
| `0x01020206` | 爬升修正 | yaw(float) + dx(s16) + dy(s16) | 爬坡姿态补偿 |
| `0x01020207` | 上R1 | `R` + `D` | 爬上 R1 |
| `0x01020208` | 撑起 | `C` + height_code | Z=归零 / L=200mm / H=400mm |
| `0x01020212` | 查询驱动速度 | `D` + `V` | 返回四轮 RPM |
| `0x01020214` | 查询转向角 | `S` + `A` | 返回四轮角度 |
| `0x010202FF` | 复位 | `R` + `S` | 无需回复 |

**速度指令 mode**：`{0,0}` 正常 · `{'C','B'}` 交叉制动 · `{'P','F'}` 预转向

#### 主控 → 执行器

| CAN ID | 功能 | 数据 | 说明 |
|--------|------|------|------|
| `0x01020501` | 使能 | `M` + enable | |
| `0x01020502` | 弹头预备 | `C` + `W` | 爪子准备姿态 |
| `0x01020503` | 弹头抓取 | `C` + `D` + side | P=手掌 Q=拳 S=矛 |
| `0x01020504` | 弹头释放 | `C` + `R` + Y/N | Y=结束 N=继续 |
| `0x01020505` | 取块预备 | `B` + `Z` | 机械臂准备姿态 |
| `0x01020506` | 取方块 | `B` + mode | L=低 / H=200mm / T=400mm / P=平地 |
| `0x01020507` | 放方块 | `B` + cell | M=中层 / T=顶层 |
| `0x01020508` | 取R1块 | `B` + `G` | |

所有命令均有对应反馈回传（`0x02xxxxxx` / `0x05xxxxxx`），实现握手确认。

### UART 串口

| 端口 | 设备 | 帧格式 | 接收方式 |
|------|------|--------|----------|
| USART1 | 激光雷达 | `FF FE` + 1B + 5×float + `AA DD` (25B) | DMA 双缓冲 + 空闲中断 |
| USART2 | 蓝牙调试 | `A5` + data + checksum + `5A` | 单字节 DMA + 校验和 |
| USART3 | R1-R2 通讯 | `FF EE` + payload + `AA DD` | 单字节中断 + RS485 半双工 |
| USART6 | 相机 | `FF FE` + 1B + 4×float + `AA DD` | 单字节 DMA 状态机 |

### R1-R2 协作

R1 通过 USART3 发送路径规划（8 路径点）、抓取指令、12 格 KFS 场地地图（0=空 / 1=R1块 / 2=R2块 / 3=假块）、对接标志（Docked / ClimbUp2R1）。

---

## 电机驱动

通过 `includes.h` 中的编译宏选择：

| 宏 | 驱动 | 总线 | 控制模式 |
|----|------|------|----------|
| `USE_DJ` | DJI M3508 | CAN 标准帧 | 级联 PID（位置→速度→电流） |
| `USE_VESC` | VESC | CAN 扩展帧 | 位置/速度/电流，PID 内置 |
| `USE_ZMDR` | ZDrive J60 | CAN 标准帧 | PVT/速度/力矩，PD 阻抗 |
| `USE_UNITREE` | Unitree GO | RS485 | PD 阻抗，17B 自定义协议 |

---

## 调试

### 蓝牙调试器 (USART2)

Unity 调试界面，帧格式 `A5` + data + checksum + `5A`。RX 47 字节 / TX 26 字节，带累加和校验。

### 帧格式

```
[A5] [Bools 4B] [Bytes 7B] [Shorts 14B] [Floats 24B] [checksum] [5A]
                              ↑ 7×short                ↑ 6×float
```

### 模式选择

| Bytes[0] | 模式 | 功能 |
|----------|------|------|
| 0 | Manual | 手柄遥控速度，手动触发执行器/底盘动作 |
| 1 | PID Tuning | 在线调参，锁点测试，轨迹测试 |
| 2 | Half-Auto | 半自动，单区域执行，手动触发动作序列 |
| 3 | Full Auto | 全自动比赛，触发 `MatchStart` |

### RX 字段完整映射

#### Bools (31 个，bit-pack 在 Bytes[0~3])

| 索引 | 名称 | Mode 0 | Mode 1 | Mode 2/3 |
|------|------|--------|--------|----------|
| 0 | chassis_en_flag | 底盘使能标志 | — | — |
| 1 | chassis_en_trig | ↑触发时发送 ChassisEnable | — | — |
| 2 | actuator_en_flag | 执行器使能标志 | — | — |
| 3 | actuator_en_trig | ↑触发时发送 Actuator_Enable | — | — |
| 4 | lock_angle | 启用 `chassisLockAngle` | — | — |
| 5 | — | — | 启用 LockPoint PID 锁点 | — |
| 6 | reset_flag | 配合 Bytes[3] 执行复位 | 同左 | 同左 |
| 7 | ascend | 发送上升指令 | — | 启动 MFcross_only |
| 8 | descend | 发送下降指令 | — | — |
| 9 | warhead_catch | 发送弹头抓取 | — | 启动 Club_only |
| 10 | warhead_ready | 发送弹头预备 | — | — |
| 11 | warhead_release | 发送弹头释放 | — | 手动设置 Docked=1 |
| 12 | kfs_ready | 发送 KFS 预备 | — | — |
| 13 | kfs_catch | 发送 KFS 抓取（模式由 Bytes[4] 决定） | — | — |
| 14 | kfs_place | 发送 KFS 放置（由 Bools[16] 选中/顶层） | — | — |
| 15 | kfs_prepare | 发送 KFS 预备取块 | — | — |
| 16 | place_mode | 0=中层 'M' / 1=顶层 'T' | — | — |
| 17 | cross_brake | 启用交叉制动 | — | — |
| 18 | set_laser_offset | 设置雷达偏移角 | — | — |
| 19 | add_path | — | — | 追加路径点到 offpath |
| 20 | kfs_place_p | 发送 KFS 放置 'P' | — | — |
| 21 | height_sel | 0=S(200mm) / 1=L(400mm) | — | — |
| 22 | nxt_move | — | — | 设置 Nxt_move 标志 |
| 23 | side_sel | 0=BLUE / 1=RED | 同左 | 同左 |
| 24 | climb_up_r1 | 发送爬 R1 指令 | — | 启动 Square9_only |
| 25 | upstand | 发送升降底盘（高度由 Bytes[5] 决定） | — | — |
| 26 | run_traj | — | 启动预定义轨迹 | — |
| 27 | slope_over | — | 触发 slopeover 爬坡 | — |
| 28 | cam_lock | — | 触发 CamLockPoint 相机锁点 | — |
| 29 | match_start | — | — | 触发 MatchStart 全自动 |
| 30 | step_mode | 切换单步模式 | — | — |

#### Bytes (7 个)

| 索引 | 名称 | 说明 |
|------|------|------|
| 0 | debugModeID | 调试模式选择 (0~3) |
| 1 | path_data | Mode 2/3 追加的路径点编号 |
| 2 | warhead_num0 | 弹头槽位编号覆盖 |
| 3 | reset_code | 复位代码（配合 Bools[6]）：0=系统复位 / 1=底盘复位 / 2=执行器复位 / 3=全复位 |
| 4 | kfs_catch_mode | KFS 抓取模式：3='H'高 / 4='T'顶 / 5='L'低 / 6='P'平地 |
| 5 | upstand_height | 升降高度码（传给 `chassis_Upstand`） |
| 6 | kfs_prepare_mode | KFS 预备取块模式 |

#### Shorts (7 个)

| 索引 | 名称 | Mode 0 | Mode 1 |
|------|------|--------|--------|
| 0 | vx | 手柄 X 速度（÷128 × 1.75 m/s） | — |
| 1 | vy | 手柄 Y 速度（÷128 × 1.75 m/s） | — |
| 2 | vw | 手柄角速度（÷128 × 300 °/s） | — |
| 3 | target_x | 测试锁点 X 坐标 (mm) | PID 锁点目标 X (mm) |
| 4 | target_y | 测试锁点 Y 坐标 (mm) | PID 锁点目标 Y (mm) |
| 5 | target_angle | 测试角度 (°) | PID 锁点目标角度 (°) |
| 6 | cam_depth | — | 相机锁点深度 (mm) |

#### Floats (6 个)

| 索引 | 名称 | Mode 0 | Mode 1 |
|------|------|--------|--------|
| 0 | pid_kp | — | 位置 PID Kp（÷1000） |
| 1 | pid_ki | — | 位置 PID Ki（÷1000） |
| 2 | pid_kd | — | 位置 PID Kd（÷1000） |
| 3 | angle_kp | — | 角度 PID Kp（÷1000） |
| 4 | angle_ki | — | 角度 PID Ki（÷1000） |
| 5 | angle_kd | — | 角度 PID Kd（÷1000） |

### 控制逻辑详解

#### 使能/失能

```
Bools[0]=1 && Bools[1]=1  →  ChassisEnable(1)   底盘使能
Bools[0]=0 && Bools[1]=1  →  ChassisEnable(0)   底盘失能
Bools[2]=1 && Bools[3]=1  →  Actuator_Enable(1)  执行器使能
Bools[2]=0 && Bools[3]=1  →  Actuator_Enable(0)  执行器失能
```

需要 Bools[x]=0 + Bools[x+1]=1 组合触发，防止误操作。

#### 复位 (Bools[6])

| Bytes[3] | 动作 |
|----------|------|
| 0 | DMA 复位 + `NVIC_SystemReset`（系统硬复位） |
| 1 | `sendChassisReset()`（仅底盘从板） |
| 2 | `Actuator_Reset()`（仅执行器从板） |
| 3 | 底盘+执行器复位 → 500ms 延时 → 系统复位 |

#### Mode 0 — 手动控制

**速度控制**：Shorts[0~2] 映射到机体坐标系速度
```
vx = Shorts[0] / 128 × 1.75 m/s
vy = Shorts[1] / 128 × 1.75 m/s
vw = Shorts[2] / 128 × 300 °/s
```
经 `Chassis_carvelSet()` 坐标变换后，由 `sendCarVel()` 发送到 CAN2。

**锁角模式** (Bools[4])：启用 `chassisLockAngle`，将 Short[5] 作为目标角度，PID 修正航向。

**底盘特殊动作**：
- Bools[7] + Enable：发送上升指令（`chassis_Ascend`），高度由 Bools[21] 选择
- Bools[8] + Enable：发送下降指令（`chassis_Descend`）
- Bools[24]：发送爬 R1 指令（`chassis_Up2R1`）
- Bools[25]：发送升降底盘（`chassis_Upstand`），高度由 Bytes[5] 决定
- Bools[30]：切换单步模式（`chassis_StepbyStep`），边沿触发

**执行器控制**（需 `Actparam.enable`）：
- Bools[9]：弹头抓取（`Actuator_WarheadCatch`），需 `grab == ACT_READY`
- Bools[10]：弹头预备（`Actuator_WarheadReady`）
- Bools[11]：弹头释放（`Actuator_WarheadRelease(0)`）
- Bools[12]：KFS 预备（`Actuator_KFSReady`）
- Bools[13]：KFS 抓取，模式由 Bytes[4] 决定（3=H / 4=T / 5=L / 6=P）
- Bools[14]：KFS 放置，Bools[16] 选中层(0='M')/顶层(1='T')
- Bools[15]：KFS 预备取块（`Actuator_KFSPrepare`），模式由 Bytes[6] 决定
- Bools[20]：KFS 放置 'P' 模式

#### Mode 1 — PID 调参

**在线调参**：Floats[0~5] 实时更新 PID 参数
```
lockPointPID[N].Kp = Floats[0] / 1000
lockPointPID[N].Ki = Floats[1] / 1000
lockPointPID[N].Kd = Floats[2] / 1000
anglePID.Kp = Floats[3] / 1000
anglePID.Ki = Floats[4] / 1000
anglePID.Kd = Floats[5] / 1000
```

**锁点测试**：Shorts[3~5] 设定目标位置/角度，Bools[5] 触发 `LockPoint`。

**相机锁点**：Bools[28] 触发 `CamLockPoint`，Shorts[6] 设定相机深度。

**爬坡测试**：Bools[27] 触发 `slopeover`。

**轨迹测试**：Bools[26] 启动预定义贝塞尔轨迹（Zone1→Zone2 入口）。

#### Mode 2 — 半自动

手动触发单个区域的状态机执行：
- Bools[9]：启动 `Club_only`（仅 Zone1）
- Bools[7]：启动 `MFcross_only`（仅 Zone2）
- Bools[24]：启动 `Square9_only`（仅 Zone3）
- Bools[19]：追加路径点（Bytes[1] → `offpath.path[rear++]`），边沿触发
- Bools[11]：手动设置 `Docked=1`（弹头对接完成）
- Bools[22]：设置 `Nxt_move` 标志

#### Mode 3 — 全自动

Mode 2 的所有功能 + 全自动启动：
- Bools[29]：触发 `Sekiro.MatchStart = true`，启动 `Sekiro_ShinobiExecution()`

### TX 遥测数据

机器人每 10ms 向调试器发送一次状态：

#### Bytes (6 个)

| 索引 | 内容 | 说明 |
|------|------|------|
| 0 | `Sekiro.Map_status.rear` | 已录入的地图格子数 |
| 1 | `Actparam.arm` | KFS 机械臂状态 (0=失能 / 1=就绪 / 2=忙 / 3=错误) |
| 2 | `Actparam.grab` | 弹头夹爪状态 (同上) |
| 3 | `Chassis.Status` | 底盘状态 (0=失能 / 1=运行 / 2=爬升中 / 3=错误) |
| 4 | `CameraRxPack.Bytes[0]` | 相机识别的方块类型 |
| 5 | `Sekiro.zone2_field` | 当前所在 Zone2 格子编号 (0~11 / 13=出口) |

#### Shorts (8 个)

| 索引 | 内容 | 单位 |
|------|------|------|
| 0 | `ChassisPosReal.x` | mm |
| 1 | `ChassisPosReal.y` | mm |
| 2 | FL 转向角 | ° |
| 3 | FR 转向角 | ° |
| 4 | BR 转向角 | ° |
| 5 | BL 转向角 | ° |
| 6 | `KFS_front.pos.x` | mm |
| 7 | `KFS_front.pos.y` | mm |

#### Floats (1 个)

| 索引 | 内容 | 单位 |
|------|------|------|
| 0 | `ChassisPosReal.angle` | ° |

---

## 编译

- **IDE**：Keil uVision 5 (MDK-ARM)
- **项目文件**：`MDK-ARM/Sekiro.uvprojx`
- **产物**：`MDK-ARM/Sekiro/Sekiro.hex`
- **清理**：`keilkill.bat`
- **CubeMX 配置**：`Sekiro.ioc`（仅修改外设时需要）

---

## 已知问题

| 文件 | 问题 |
|------|------|
| `Sekiro.c` `Match_UptoR1` | `ClimbUp2R1` 在设置后立即被清除（WIP） |
| `Sekiro.c` `Match_ChassisSearchRoutes` | 路径搜索逻辑未实现 |
| `FD_Canqueue.c` `CAN_DequeueRx` | Front 指针递增两次，跳过队列元素 |
| `Actuator.c` `Actuator_KFSGrabR1Block` | 队列满检查逻辑反转 |

---

## 协议文档

- `R2终代车通讯协议V1.03.md` — 终代车完整 CAN 协议
- `R2二代车通讯协议V1.02.xlsx` — 二代车协议（历史参考）

---

## 许可证

[MIT License](LICENSE) · Copyright 2025 WHUROBOCON
