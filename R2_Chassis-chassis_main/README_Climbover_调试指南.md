# 底盘爬台阶（Climbover）调试指南

## 文件概览

本调试指南涉及的四个核心文件：

| 文件 | 路径 | 职责 |
|------|------|------|
| **Climbover.h** | `Chassis/inc/Climbover.h` | 枚举、结构体、全局变量声明、函数接口声明 |
| **Climbover.c** | `Chassis/src/Climbover.c` | 状态机实现（上台阶/下台阶/站立/R1）、轨迹规划、稳定判定 |
| **myostask.h** | `Control/inc/myostask.h` | 任务函数声明、共享变量声明 |
| **myostask.c** | `Control/src/myostask.c` | FreeRTOS 任务定义（主控/通信/轨迹规划/LED） |

---

## 一、调试模式 vs 自动模式

全局变量 `DEBUG_CHANGE`（定义在 `Climbover.c`）控制运行模式：

- **`DEBUG_CHANGE = 0`（默认）**：自动模式。状态机启动后自动按流程推进，每一步条件满足后自动切换到下一步。
- **`DEBUG_CHANGE = 1`**：手动单步调试模式。启动后停留在 `Waiting_For_Init` 状态，通过外部命令调用 `Climb_Ascend_DebugNext()` / `Climb_Descend_DebugNext()` 触发下一步。

> 调试建议：首次调试时先用 `DEBUG_CHANGE = 1` 模式单步验证每个动作是否正常，再切到自动模式。

---

## 二、核心调参结构体 `ClimbConfig`

所有关键参数集中在全局结构体 `ClimbCfg` 中（定义在 `Climbover.c`），可直接在代码中修改初始值，或通过串口调试工具在运行时修改。

### ⚠️ 两处修改入口的区别

参数有两种修改位置，预期效果不同：

---

#### (A) 结构体初始化默认值（`ClimbConfig ClimbCfg = { ... }`）

这段位于 `Climbover.c` 全局变量区，是所有参数的**默认值**（默认按 400mm 台阶设定）。如果**不调用** `Climb_SetHeight()`，上台阶时会直接使用这里面的值。

**应在此处修改的参数**（`Climb_SetHeight` **不会复写**的参数）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `.ascend_R1_angle` | 3060.f | 上 R1 抬腿角度，只在 `Climb_UpToR1` 中使用 |
| `.ascend_chassis_vel` | 0.55f | 上台阶底盘前进速度（若用自动模式，会被 `Climb_SetHeight` 覆盖，见下文） |
| `.front_sensor_threshold` | 60 | 前 TinyF 传感器阈值 |
| `.R1_wheel_delta_angle` | 1250 | 上 R1 后轮前进角度 |
| `.R1_wheel_track_ticks` | 2500 | 上 R1 后轮轨迹总周期数 |

> 但注意 **`.ascend_chassis_vel`** 比较特殊：它在默认初值中为 0.55，而在 `Climb_SetHeight(ClimbHeight_200mm/400mm)` 中**没有**被复写，因此它的值**始终是 0.55**，不受高度切换影响。如果想对不同高度用不同的 `ascend_chassis_vel`，需在 `Climb_SetHeight` 中手动添加赋值。

---

#### (B) `Climb_SetHeight()` 函数内

调用 `Climb_SetHeight(h)` 会覆盖 `ClimbCfg` 中对应字段。**这里修改的值会随高度切换改变**。

**不同高度的参数对照表**（`Climb_SetHeight` 中赋值的参数）：

| 参数 | 200mm 值 | 400mm 值 | Zero 值 | 说明 |
|------|----------|----------|---------|------|
| `ascend_track_ticks` | 110 | 180 | — | 上台阶腿部轨迹周期 |
| `ascend_stand_angle` | 1650 | 3060 | 100 | 站立目标角度 |
| `wheel_speed` | 220 | 120 | — | 车轮电机转速 |
| `descend_track_ticks` | 130 | 200 | — | 下台阶轨迹周期 |
| `ascend_lift_angle` | 1650 | 3000 | 100 | 上台阶抬腿角度 |
| `descend_put_angle` | 1530 | 2900 | — | 下台阶放腿角度 |
| `ascend_leg_acc_limit` | 100 | 200 | 100 | 上台阶腿加速度 |
| `ascend_leg_vel_limit` | 15 | 20 | 15 | 上台阶腿速度 |
| `descend_leg_acc_limit` | 300 | 650 | — | 下台阶腿加速度 |
| `descend_leg_vel_limit` | 30 | 60 | — | 下台阶腿速度 |
| `retract_leg_acc_limit` | 200 | 450 | — | 收腿加速度 |
| `retract_leg_vel_limit` | 20 | 45 | — | 收腿速度 |
| `chassis_speed` | 0.78 | 0.68 | — | 底盘前进速度 |

**注意**：
- 表中标 "—" 表示该高度分支中**未赋值**该参数，会保留结构体初始化时的默认值
- `totalStep` 仅在 `Zero` 分支中被赋值为 200，其他分支不会修改它

---

### 如何选择修改位置？

- **如果某个参数**对不同台阶高度的值不同 → 在 `Climb_SetHeight` 中各分支分别修改
- **如果某个参数**对所有高度都一样（或你不想随高度切换而变）→ 在结构体初始化默认值中修改
- **最佳实践**：想改 200mm 的上台阶抬腿角度 → 找 `Climb_SetHeight` 里的 `ClimbHeight_200mm` 分支，改 `.ascend_lift_angle = 你的值`

### 2.1 角度参数

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `ascend_lift_angle` | 上台阶时四条腿一致抬升到的目标角度 | 值越大腿抬得越高。200mm 约 1650，400mm 约 3000。调太大可能翻车，调太小前轮卡台阶。 |
| `ascend_stand_angle` | 站立/起身时的目标角度 | 200mm 约 1650，400mm 约 3060。 |
| `ascend_R1_angle` | 上 R1 时的目标角度 | 约 3060。 |
| `descend_put_angle` | 下台阶时腿向外伸出的目标角度 | 前腿放太短够不到地，放太长会撑起车身。200mm 约 1530，400mm 约 2900。 |

### 2.2 速度/加速度参数

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `ascend_leg_vel_limit` | 上台阶时腿运动速度上限 | 太大容易抖动/过冲，太小动作慢。200mm 用 15，400mm 用 20。 |
| `ascend_leg_acc_limit` | 上台阶时腿加速度上限 | 太大车身晃动明显。200mm 用 100，400mm 用 200。 |
| `descend_leg_vel_limit` | 下台阶时腿运动速度上限 | 下台阶可以稍快，200mm 用 30，400mm 用 60。 |
| `descend_leg_acc_limit` | 下台阶时腿加速度上限 | 200mm 用 300，400mm 用 650。 |
| `retract_leg_vel_limit` | 收腿时的速度上限 | 收腿比抬腿可稍快，200mm 用 20，400mm 用 45。 |
| `retract_leg_acc_limit` | 收腿时的加速度上限 | 200mm 用 200，400mm 用 450。 |

**调试原则**：
- 上台阶时速度/加速度**偏低**（防止腿撞击台阶）
- 下台阶时速度/加速度可以**稍高**（需要快速伸出撑地）
- 收腿时速度/加速度适中（既要快又不能太猛）

### 2.3 车轮 & 底盘速度

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `wheel_speed` | 车轮电机转速（角度制单位） | 200mm 用 220，400mm 用 120。值越大前进越快。调到前轮刚好搭上台阶为宜。 |
| `chassis_speed` | 底盘麦轮整体速度（m/s） | 200mm 用 0.78，400mm 用 0.68。配合 wheel_speed 协调。 |
| `ascend_chassis_vel` | 上台阶时底盘前进速度 | 约 0.55，速度太快会撞台阶。 |

### 2.4 轨迹周期

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `ascend_track_ticks` | 上台阶腿部轨迹总周期数 | 一个 tick = 5ms。200mm 约 110tick(550ms)，400mm 约 180tick(900ms)。调大动作变慢变柔，调小动作变快变猛。 |
| `descend_track_ticks` | 下台阶腿部轨迹总周期数 | 200mm 约 130，400mm 约 200。 |
| `R1_wheel_track_ticks` | 上R1后轮轨迹总周期数 | 约 2500tick，用于防止电机力矩不足刹停不稳。 |

### 2.5 传感器阈值

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `front_sensor_threshold` | 前气缸高度突变阈值 | 约 60。当前方 TinyF 传感器读数落入阈值范围内时，判断前轮已搭上台阶。 |

### 2.6 R1 专用

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `R1_wheel_delta_angle` | 上R1后轮需要转动的总角度 | 约 1250。调大前进距离增加。 |

### 2.6 R1 专用参数

| 参数名 | 说明 | 调试要点 |
|--------|------|----------|
| `solenoid_flag` | 控制气缸和辅助轮，`FUNZHU_FRONT=0x05`=前气缸+辅助轮，`FUNZHU_BACK=0x09`=后气缸+辅助轮。 | |

---

## 三、台阶高度切换 `Climb_SetHeight(ClimbHeight h)`

通过调用 `Climb_SetHeight()` 或直接修改 `climb_height` 变量，可一键切换整套参数：

```c
Climb_SetHeight(ClimbHeight_200mm);   // 200mm 台阶参数
Climb_SetHeight(ClimbHeight_400mm);   // 400mm 台阶参数
Climb_SetHeight(ClimbHeight_Zero);    // 零高度（调试或站立用）
```

**注意**：实际调用 `Climb_SetHeight` 后，它会覆盖 `ClimbCfg` 中的对应字段。如果你单独修改了某个参数后又调了这个函数，之前的手改值会被覆盖。

---

## 四、稳定判定函数 `Climb_Done_Stable`

状态机中跳转条件大量使用该函数做防抖处理：

```c
bool Climb_Done_Stable(bool condition, uint16_t need, uint16_t need2);
```

- `condition`：要判定的条件
- `need`：条件连续成立多少次才认为真正成立（防误判）
- `need2`：条件不成立累计多少次才清空成功计数（防短暂抖动）

**预定义等待时间**（定义在 `Climbover.h`）：

| 宏 | 实际时间 | 用途场景 |
|----|----------|----------|
| `STABLE_WAIT_TINY` | 50ms | 快速判稳（角度到位、传感器跳变） |
| `STABLE_WAIT_SHORT` | 100ms | 短时等待（过渡延时） |
| `STABLE_WAIT_MEDIUM` | 300ms | 适中等待（降底盘、收腿等） |
| `STABLE_WAIT_LONG` | 1000ms | 长时间等待（站立等高行程动作） |

**调试建议**：如果某个步骤总是提前跳走或卡死不动，检查对应位置的 `Climb_Done_Stable` 调用，可能是 need/need2 设置不合理。

---

## 五、传感器说明（重要）

- **超声波**：代码中 `UltrasonicTask` 虽存在但**实际未使用**，硬件上也未安装超声波传感器，调试时请忽略该任务。状态机中也没有使用超声波数据。
- **TinyF 光电传感器（实际使用）**：状态机使用 `TinyF_CTX[TINYF_SOL]`（前）、`TinyF_CTX[TINYF_MID]`（中）、`TinyF_CTX[TINYF_FRONT]`（前远）、`TinyF_CTX[TINYF_BACK]`（后）这四个光电传感器来判断各车轮相对台阶的位置。调试时请确认这些传感器已安装且读数正确。

---

## 六、两个轨迹规划任务

### 6.1 `guiji` 任务（核心）

5ms 周期运行，负责两件事：

1. **腿部轨迹**（`Track_flag == 1` 时触发）
   - 调用 `Leg_SinHalfMove` 对四条腿做平滑 ease-in-out 插值
   - `totalStep` 控制总周期数，`leg_track_target` 为目标角度
   - 四个电机全部完成后清除 `Track_flag`

2. **车轮轨迹**（`Wheel_Track_Flag == true` 时触发）
   - 调用 `Set_Wheel_Pos` 做 smoothstep 缓冲
   - 完成后置 `UP_OK = 1`，通知状态机条件满足



## 七、状态机流程速查

### 上台阶流程（`Climb_Ascend_Step`）

```
Waiting_For_Init
    → Leg_All_Down:         设置腿的限幅，四条腿同步抬升到 ascend_lift_angle
    → Find_Position:        打开前气缸+辅助轮，车轮前进，等前传感器触发
    → Front_Leg_Up:         前腿收回，底盘继续前进，等中传感器确认后轮到位
    → Go_Forward:           短时前进过渡（100ms）
    → Back_Leg_Up:          收后腿，关闭气缸，停车，回复主控 OK
```

### 下台阶流程（`Climb_Descend_Step`）

```
Waiting_For_Init
    → On_Position:          设置腿的限幅，底盘前进，开后气缸，等前传感器触发
    → Front_Leg_Down:       前腿伸出到 descend_put_angle（短时等待）
    → Find_Position:        底盘继续前进，等后传感器触发
    → Back_Leg_Down:        后腿伸出到 descend_put_angle，确认到位后关气缸
    → Go_Forward:           前进下台阶
    → Leg_All_Up:           等待 300ms，收四条腿到 100°，回复主控 OK
```

### 站立流程（`Climb_UpStand`）

```
一次性动作：osDelay(10) → 四条腿抬升到 ascend_stand_angle → 等待 1s → 回复 OK
```

### 上 R1 流程（`Climb_UpToR1`）

```
Waiting_For_Init
    → Leg_All_Down:         后轮切位置模式，四腿抬升到 ascend_R1_angle
    → Find_Position:        后轮走 R1_wheel_delta_angle 的平滑轨迹
    → Front_Leg_Up:         等待 50ms
    → Go_Forward:           等待 50ms
    → Back_Leg_Up:          等待 50ms → 回复主控 OK
```

---

## 八、调试常见问题


---
