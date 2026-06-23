# AGENTS.md

本文件是当前 STM32 + 树莓派自动采摘机器人项目的长期记忆文件，供 VSCode / Codex / Cursor / 通义灵码等写代码 AI 使用。

任何 AI 在修改本工程代码前，必须先阅读本文件，并按“每次代码修改规则”执行。不要私自改变已确认的硬件参数；不确定的底层函数、串口句柄、舵机编号、机械参数必须用 TODO 标出。

## 1. 项目目标

本项目用于 RAICOM 睿抗慧眼识果赛项的自动采摘机器人。

机器人需要完成：

1. 自动从启动区移动到果树旁边。
2. 每棵树设置左侧观察点和右侧观察点。（此功能已删除）
3. 小车到达果实采摘区域后开始缓慢前进等待树莓派下发看到果实的命令然后开始运行xyz轴捕捉苹果。
4. STM32 先发送 `WATCH` 让树莓派连续识别；收到 `HIT` 停车后再发送 `SCAN` 请求最终抓取坐标。
5. 树莓派使用深度相机 + ONNX YOLO 识别苹果。
6. YOLO 检测苹果位置；大小由深度相机估算真实直径后判断。
7. 大小苹果由深度相机估算真实物理直径判断。
8. 苹果只有两类：BIG 大苹果约 85mm，SMALL 小苹果约 50mm。
9. 树莓派完成相机坐标到机械臂坐标的映射。
10. 树莓派返回机械臂绝对坐标 x/y/z 给 STM32。
11. STM32 控制 XYZ 三轴到达抓取点。
12. 夹爪中心对准苹果中心时，剪刀已经机械对准挂苹果的绳子。
13. 夹爪夹住苹果，当前代码剪刀执行一次剪切确认，剪刀最终保持张开。
14. 剪刀最终张开后不需要 Z 轴上抬，直接 Y 轴后撤。
15. 抓取结构通过舵机控制左放/右放，实现大小果分类，左放大苹果，右放小苹果。

## 2. 硬件架构

| 模块 | 配置 |
|---|---|
| 下位机 | STM32F407VET6 + FreeRTOS |
| 上位机 | 树莓派 |
| 视觉 | 深度相机 + ONNX YOLO |
| 电机驱动 | Emm_V5 闭环步进驱动 |
| 底盘 | 带控制器的步进电机底盘 |
| 机械臂 | XYZ 三轴 |
| 末端执行器 | 夹爪 + 剪刀 + 分类舵机 |
| 构建工具 | Keil MDK-ARM v5 |
| 调试器 | ST-Link |

当前工程已有外设和模块包括：激光雷达、SBUS、OLED、舵机、SC16IS752、Emm_V5、Arm 归零、base_control、Usart_to_Pi 等。

## 3. 软件架构

### STM32 侧

当前工程是 STM32CubeMX + HAL + FreeRTOS 结构：

- `Core/Inc`：头文件。
- `Core/Src`：源文件。
- `Drivers`：STM32 HAL / CMSIS。
- `Middlewares/Third_Party/FreeRTOS`：FreeRTOS 内核。
- `MDK-ARM/v1.0.uvprojx`：Keil 工程。

推荐 STM32 业务文件结构：

- `game_task.c / game_task.h`：比赛主流程。
- `arm_motion.c / arm_motion.h`：XYZ 绝对位置控制。
- `fruit_actuator.c / fruit_actuator.h`：夹爪、剪刀、分类舵机。
- `fruit_pick.c / fruit_pick.h`：单果抓取流程。
- `pi_protocol.c / pi_protocol.h`：上下位机通信协议。
- `tree_view.c / tree_view.h`：底盘移动到果实采摘区域或赛道关键点。
- `pick_loop.c / pick_loop.h`：连续巡航期间多次停车抓取多个果。
- `calib_test.c / calib_test.h`：硬件标定和测试。

当前工程中已经存在：

- `Core/Inc/arm_motion.h`
- `Core/Src/arm_motion.c`
- `Core/Inc/Usart_to_Pi.h`
- `Core/Src/Usart_to_Pi.c`

注意：当前 `arm_motion` 旧版本里存在相对移动接口。后续修改必须按本文件的新规则迁移到“绝对位置模式”，不要继续扩展相对运动封装。

### 树莓派侧

推荐树莓派代码结构：

- `main.py`
- `camera.py`
- `yolo_detector.py`
- `fruit_size.py`
- `coordinate_map.py`
- `target_selector.py`
- `serial_protocol.py`
- `calibration.py`
- `camera_to_arm.json`

当前工程中正在调试使用的树莓派视觉脚本路径：

- `E:\Embedded\microcomputer\26RuiKang\V3.1\V1.1\CODE\v1.0\树莓派视觉代码\v2.4.py`

## 4. FreeRTOS 任务规则

继续使用 `StartDefaultTask` 作为 GameTask，也就是比赛主流程入口。

`StartDefaultTask` 不要写大量动作细节，只调用：

```c
void Game_Init(void);
void Game_Run(void);
```

传感器任务可以继续存在，例如激光、IMU、限位、OLED、串口接收任务。

必须遵守：

1. 只有 `StartDefaultTask` / `Game_Run` 可以控制底盘、XYZ、夹爪、剪刀、分类舵机。
2. 其他任务只能读取传感器或接收数据，不能主动控制电机。
3. 不要创建多个运动控制任务。
4. 不要让多个任务同时控制同一个电机。
5. FreeRTOS 运行后使用 `osDelay()`；初始化阶段才允许使用 `HAL_Delay()`。
6. CubeMX 生成文件中的自定义逻辑必须放在 `USER CODE` 块内。

### 4.1 OLED 显示任务规则

OLED 屏幕刷新必须由一个显示任务统一执行，当前优先使用 `StartOledTask`。

必须遵守：

1. 只有 `StartOledTask` 或后续明确指定的专门 UI 任务可以直接刷新 OLED。
2. 其他任务不要直接频繁调用 `OLED_ClearBuffer()`、`OLED_DrawString()`、`OLED_DrawNum()`、`OLED_Flush()`。
3. `StartDefaultTask` / `Game_Run` 等业务任务如果需要显示状态，必须通过 FreeRTOS 内置同步机制把状态传给 OLED 任务。
4. 简单状态推荐使用 `volatile` 状态变量、`EventGroup` 或 `Task Notification`。
5. 复杂文本、多字段状态或多模块 UI 请求推荐使用 `Queue`。
6. 不要让两个任务同时改 OLED buffer 或同时 `OLED_Flush()`，避免显示撕裂、覆盖和 I2C/DMA 竞争。
7. OLED 文本必须控制在当前屏幕范围内：SSD1306 128x64，8x16 字体时最多 4 行、每行 16 个 ASCII 字符。

### 4.2 按键读取和人工确认规则

按键用于启动确认、运动确认或危险动作确认时，必须使用消抖逻辑。

必须遵守：

1. 优先使用 `Button_ReadDebounce()` 或封装了同等消抖等待的按键函数。
2. 不要直接用裸 `HAL_GPIO_ReadPin()` 作为启动、运动、抓取或倒果等动作的确认条件。
3. 等待人工确认时，应先等待按键处于释放状态，再等待一次稳定按下，避免上电长按或抖动导致误启动。
4. 当前 `button.h` 已封装的按键是 `KEY1~KEY4 = PE3~PE6`。
5. 当前 `PE2` 在 `gpio.c` 中配置为输出，不是按键输入；除非实物和 CubeMX/GPIO 代码都已确认并同步更新本文件，否则不要把 `PE2` 当作按键使用。

## 5. 已有底层函数

已确认存在的 Emm_V5 驱动函数：

```c
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
```

已确认存在的机械臂归零函数：

```c
int Arm_Init_AllParallel(void);
```

已确认存在的底盘函数：

```c
int Base_RotateToYaw(float target_yaw, int16_t max_speed_rrp, float kp, float kd);
int Base_FollowWall(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float kd);
int Base_ForwardUntilFrontDistance(uint16_t stop_dist_mm, int16_t forward_speed_rrp);
int Base_ForwardDistanceCmHoldYaw(float distance_cm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd);
int Base_ForwardDistanceCmHoldYawWatchPi(float distance_cm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd, uint8_t tree_id, TreeViewId_t view_id, float *traveled_cm_out);
```

已确认存在的树莓派串口基础封装：

```c
void PiUart2_Init(void);
void PiUart2_StartRx(void);
bool PiUart2_GetFrame(uint8_t *out, uint16_t out_len, uint16_t *frame_len);
bool PiUart2_Send(const uint8_t *data, uint16_t len);
bool PiUart2_SendByte(uint8_t data);
```

已确认树莓派串口当前使用 `huart2`。若后续硬件改线，必须更新本文件和代码。

如果 AI 不确定某个底层函数是否存在，必须先搜索代码确认；仍不确定时用 TODO 标出，不允许自己编造函数名。

## 6. 已确认的电机轴信息

XYZ 三轴使用 Emm_V5 绝对位置模式。

| 轴 | 电机 ID | 正方向 dir | 换算关系 | 每 mm 脉冲 |
|---|---:|---:|---|---:|
| X | 7 | 1 | 3200 脉冲 = 4cm = 40mm | 80 |
| Y | 6 | 1 | 3200 脉冲 = 4cm = 40mm | 80 |
| Z | 5 | 0 | 3200 脉冲 = 1cm = 10mm | 320 |

已确认 XYZ 机械行程范围：

| 轴 | 最小值 | 最大值 |
|---|---:|---:|
| X | `0mm` | `220mm` |
| Y | `0mm` | `480mm` |
| Z | `0mm` | `400mm` |

已确认 `Arm_RetractSafe()` 安全回收点：

```c
Arm_MoveToPoint(0, 40, 0);
```

注意：`ARM_SAFE_X_MM / ARM_SAFE_Y_MM / ARM_SAFE_Z_MM` 如果后续使用，应表示安全回收点坐标，不表示最大行程。最大行程应使用 `ARM_MOTION_X_MAX_MM / ARM_MOTION_Y_MAX_MM / ARM_MOTION_Z_MAX_MM`。

已确认正方向调用：

```c
Emm_V5_Pos_Control(7, 1, 200, 20, clk, true, false); // X 正方向绝对控制
Emm_V5_Pos_Control(6, 1, 200, 20, clk, true, false); // Y 正方向绝对控制
Emm_V5_Pos_Control(5, 0, 200, 20, clk, true, false); // Z 正方向绝对控制
```

推荐宏名：

```c
#define X_CLK_PER_MM 80u
#define Y_CLK_PER_MM 80u
#define Z_CLK_PER_MM 320u
```

## 6.1 已确认的末端执行器舵机信息

夹爪、剪刀、分类舵机使用 `servo.c / servo.h` 中的 `Servo_SetAngle()` 控制。

| 执行器 | 舵机编号 | 已确认角度 |
|---|---|---|
| 夹爪 | `SERVO_2` | 打开 `255`，闭合 `100` |
| 剪刀 | `SERVO_1` | 打开 `270`，剪断 `60` |
| 分类舵机 | `SERVO_3` | 中间 `135`，原大果仓/当前小果暂存区 `270`，原小果仓/当前大果暂存区 `0` |
| 原大果篮/当前小果暂存区 | `SERVO_4` | 垂直避让 `45`，装载 `135`，倒出 `65` |
| 原小果篮/当前大果暂存区 | `SERVO_5` | 垂直避让 `215`，装载 `125`，倒出 `170` |

注意：

1. 以上舵机编号和角度已经过现场确认，不要再标为 TODO。
2. 后续如机械结构调整导致角度变化，必须同步修改 `fruit_actuator.c` 和本文件。
3. 上电 XYZ 回零前，原大果篮/当前小果暂存区必须先到 `45` 度垂直避让，原小果篮/当前大果暂存区必须先到 `215` 度垂直避让。
4. XYZ 回零完成并且 Y 轴移动到 `40mm` 后，原大果篮/当前小果暂存区再回 `135` 度装载状态，原小果篮/当前大果暂存区再回 `125` 度装载状态。

## 6.2 已确认的大小果投放准备点

抓取完成并完成 Y 轴后撤后，XYZ 需要先移动到对应果仓的投放准备点，再执行分类舵机动作和夹爪释放。

| 果子类型 | XYZ 投放准备点 | 分类舵机动作 |
|---|---|---|
| 大果 `FRUIT_BIG` | `Arm_MoveToPoint(220, 40, 250)` | `Servo_SetAngle(SERVO_3, 0)` |
| 小果 `FRUIT_SMALL` | `Arm_MoveToPoint(40, 40, 250)` | `Servo_SetAngle(SERVO_3, 270)` |

注意：

1. 大果和小果的 XYZ 投放准备点不同，不要共用一个固定 `DROP_X/Y/Z`。
2. 以上点位已经过用户实车确认，完全安全；后续若机械仓位调整，必须同步修改 `fruit_pick.c` 和本文件。
3. 当前大小果暂存区已整体互换：大果使用原小果暂存区，分类舵机角度 `0`；小果使用原大果暂存区，分类舵机角度 `270`。
4. 当前倒果逻辑也随暂存区整体互换：`FruitBasket_DumpBig()` 操作原小果篮 `SERVO_5`，`FruitBasket_DumpSmall()` 操作原大果篮 `SERVO_4`。

## 7. XYZ 绝对位置控制规则

当前项目决定使用 Emm_V5 的绝对位置模式，不使用相对位置模式封装 XYZ。

初始化流程：

1. 调用 `Arm_Init_AllParallel()`，让 XYZ 三轴回到限位/碰撞零点。
2. 回零完成后调用：

```c
Emm_V5_Reset_CurPos_To_Zero(7);
Emm_V5_Reset_CurPos_To_Zero(6);
Emm_V5_Reset_CurPos_To_Zero(5);
```

3. 此后所有 XYZ 运动都使用绝对位置模式：

```c
Emm_V5_Pos_Control(..., true, false);
```

其中 `raF = true` 表示绝对位置模式。

必须遵守：

1. 不要使用相对位置模式。
2. 不要写 `Axis_MoveRelMm`。
3. 所有 XYZ 输入单位是 mm。
4. `arm_motion.c` 内部负责 mm 转 clk。
5. 所有行程限制用宏定义。
6. 每个运动函数必须返回错误码。
7. 第一版一轴一轴运动，不要多轴同步。
8. 不要创建新的 FreeRTOS 任务。
9. 不要使用 `malloc`。
10. 除上电复位 XYZ 坐标过程外，后续普通运动的 Y 轴绝对坐标必须大于等于 40mm。
11. 上电 XYZ 回零完成后，必须立即把 Y 轴移动到 `40mm`。
12. 调用 `Arm_MoveToPoint()` 时，如果当前 Y 轴绝对坐标小于 40mm，必须先把 Y 轴移动到 40mm，再开始移动 X/Z 等其它轴；目标 Y 轴小于 40mm 时应返回错误，不允许继续收回到 40mm 以下。

推荐 `arm_motion.c / arm_motion.h` 接口：

```c
typedef struct {
    float x;
    float y;
    float z;
} ArmPoint_t;

typedef enum {
    ARM_AXIS_X = 0,
    ARM_AXIS_Y,
    ARM_AXIS_Z
} AxisId_t;

int Arm_HomeAndZero(void);
int Arm_ZeroAfterHome(void);
int Axis_MoveToAbsMm(AxisId_t axis, float target_mm);
int Arm_MoveToPointAbs(ArmPoint_t target);
int Arm_MoveYToAbs(float y_mm);
int Arm_RetractSafe(void);
bool Arm_IsPointReachable(ArmPoint_t p);
```

## 8. 树莓派视觉职责

树莓派负责：

1. 接收 STM32 的 `WATCH` / `WATCH_STOP` / `SCAN` 请求。
2. `WATCH` 模式下持续采集 RGB + Depth 并连续识别。
3. 使用 ONNX YOLO 检测苹果 bbox。
4. YOLO 可检测苹果位置；大小分类由深度相机估算真实直径后判断。
5. 在 bbox 内结合深度相机估算苹果真实物理直径。
6. 根据直径判断 BIG / SMALL。
7. 计算苹果中心的相机 3D 坐标。
8. 通过标定矩阵或映射关系转换成机械臂绝对坐标 `arm_x / arm_y / arm_z`。
9. 在 `WATCH` 模式下，只有目标进入可抓范围时主动发送 `HIT`，用于通知 STM32 停车。
10. 停车后收到 `SCAN` 时重新取帧，返回当前最适合抓取的一个最终目标。
11. 如果没有合适目标，返回 `NONE`。

大小判断建议：

- `diameter_mm >= 70`：BIG。
- `diameter_mm <= 62`：SMALL。
- `62 < diameter_mm < 70`：UNKNOWN，不抓取。

树莓派负责大小果判断和坐标映射，STM32 不做复杂视觉映射，也不根据 `diameter` 重新判断大果/小果。

## 9. STM32 下位机职责

STM32 负责：

1. 运行比赛主流程。
2. 控制底盘到达果实采摘区域。
3. 调用 `Game_CreepWatchAndPick()` 让小车慢速保持航向前进。
4. 向树莓派发送 `WATCH`，前进过程中轮询接收 `HIT`。
5. 收到合法 `HIT` 后立即停车，并发送 `WATCH_STOP`。
6. 停稳后发送 `SCAN`，接收树莓派返回的最终 `FRUIT` 或 `NONE`。
7. 校验目标类型、坐标范围、直径、score。
8. 调用 `Arm_MoveToPointAbs()` 或当前兼容接口控制 XYZ 到达绝对坐标。
9. 根据 Pi 返回的 `fruit.type` 执行大果/小果对应投放准备点和分类舵机动作。
10. 控制夹爪、剪刀和分类舵机。
11. 管理传感器、限位、OLED、调试显示。

STM32 不负责：

- 不跑 YOLO。
- 不估算苹果直径。
- 不根据 `diameter` 再次判断 BIG / SMALL。
- 不做相机坐标到机械臂坐标的复杂映射。
- 不一次处理多个视觉目标。

## 10. 上下位机通信协议

当前使用简单 ASCII 文本协议，并已增加连续巡航识别流程。

串口建议：

- 115200
- 8N1
- 每条消息以 `\n` 结尾
- 不使用中文
- 第一版不要 CRC

STM32 -> 树莓派：

```text
PING,seq\n
WATCH,seq,tree_id,view_id\n
WATCH_STOP,seq\n
SCAN,seq,tree_id,view_id\n
```

`view_id` 使用 `L` 或 `R`。

树莓派 -> STM32：

```text
PONG,seq\n
WATCHING,seq\n
HIT,seq,type,x,y,z,diameter,score\n
FRUIT,seq,type,x,y,z,diameter,score\n
NONE,seq\n
ERR,seq,error_code\n
```

示例：

```text
STM32: WATCH,100,0,L\n
PI:    WATCHING,100\n
PI:    HIT,100,BIG,132,190,86,83,91\n
STM32: WATCH_STOP,100\n
STM32: SCAN,101,0,L\n
PI:    FRUIT,101,BIG,128,188,85,83,93\n
```

含义：

- `type = BIG`
- `x = 128mm`
- `y = 188mm`
- `z = 85mm`
- `diameter = 83mm`
- `score = 93`

注意：

1. x/y/z 是机械臂绝对坐标，单位 mm。
2. x/y/z 已经由树莓派完成相机坐标到机械臂坐标映射。
3. `HIT` 只用于通知 STM32 停车，不作为最终抓取坐标。
4. STM32 停稳后必须重新发送 `SCAN`，最终抓取只使用 `FRUIT` 的坐标。
5. STM32 收到最终坐标后调用 `Arm_MoveToPointAbs()` 或当前兼容接口。
6. `type` 由树莓派决定，STM32 不根据 `diameter` 再次判断大小果，只按 `type` 执行投放。

当前 `pi_protocol.c / pi_protocol.h` 接口：

```c
int Pi_Ping(uint16_t seq, uint32_t timeout_ms);
int Pi_StartWatch(uint8_t tree_id, TreeViewId_t view_id, uint16_t *watch_seq, uint32_t timeout_ms);
int Pi_PollWatchHit(uint16_t watch_seq, FruitTarget_t *out, uint32_t timeout_ms);
int Pi_StopWatch(uint16_t watch_seq);
int Pi_RequestBestFruit(uint8_t tree_id, TreeViewId_t view_id, FruitTarget_t *out, uint32_t timeout_ms);
int Pi_ParseLine(const char *line, uint16_t expected_seq, FruitTarget_t *out);
```

推荐返回值：

```c
typedef enum {
    PI_OK = 0,
    PI_ERR_TIMEOUT = -1,
    PI_ERR_PARSE = -2,
    PI_ERR_NONE = -3,
    PI_ERR_REMOTE = -4,
    PI_ERR_OUT_OF_RANGE = -5,
    PI_ERR_PARAM = -6
} PiResult_t;
```

`pi_protocol` 模块要求：

1. 不要使用 `malloc`。
2. 使用固定长度 char buffer，例如 128 字节。
3. 所有等待必须有 timeout。
4. 不要控制 XYZ。
5. 不要创建新的 FreeRTOS 任务。
6. 当前底层串口可优先复用 `Usart_to_Pi`，已确认使用 `huart2`。
7. 解析整数必须使用 `strtol()`，不要使用 `atoi()`，必须支持负号。
8. 如果 `HIT` 或 `FRUIT` 中 x/y/z 为负数、Y 小于 40mm，或坐标超出机械臂范围，必须返回 `PI_ERR_OUT_OF_RANGE`。
9. 越界 `HIT` 不停车；越界 `FRUIT` 不进入抓取流程。

调试阶段 `SCAN` 等待时间建议不低于 `10000ms`。树莓派需要相机取帧、深度处理、YOLO 推理和坐标映射，3 秒可能导致 STM32 已经超时退出，但树莓派随后才发出 `FRUIT`。

当前连续巡航流程：

1. STM32 调用 `Game_CreepWatchAndPick()`。
2. 底盘调用 `Base_ForwardDistanceCmHoldYawWatchPi()`，保持绝对航向慢速前进。
3. 该底盘函数内部发送 `WATCH`，并在控制周期内轮询 `HIT`。
4. 收到合法 `HIT` 后底盘立即停车，返回本次估算前进距离。
5. `Game_CreepWatchAndPick()` 累加距离；若总距离未到设定值，则抓取/跳过后继续前进。
6. 停车后发送 `SCAN` 重拍，收到最终 `FRUIT` 后才执行 `FruitPick_PickOne()`。

## 11. 抓取动作流程

机械结构已经保证：夹爪中心对准果实中心时，剪刀已经对准挂果子的绳子。

因此软件不需要计算单独的剪切点。

抓取流程：

1. `Gripper_Open()`
2. `Cutter_Open()`
3. 计算 grip 点，也就是苹果中心对应的机械臂绝对坐标。
4. 不再计算预备点，不再先移动到 `pre`。
5. `Arm_MoveToPointAbs(grip)`，直接移动到树莓派返回的苹果中心抓取点。
6. 必须等待 XYZ 彻底到达抓取点并稳定后，才能 `Gripper_Close()`。
7. `Gripper_Close()`
8. `Cutter_Cut()`
9. `Cutter_Open()`，当前代码执行一次剪切，最终保持剪刀张开。
10. 剪刀最终张开后不抬 Z，直接 `Arm_MoveYToAbs(40)` 后撤到 Y 轴安全线。
11. 移动到大小果对应投放准备点。
12. `Sorter_DropByType(fruit.type)`
13. `Arm_RetractSafe()`

注意：当前 Emm_V5 还没有真实“运动完成反馈”，`arm_motion.c` 里是估算等待。抓取点到位后必须保留额外稳定等待；如果实车出现未完全到位就夹爪，优先增大 `FRUIT_PICK_GRIP_ARRIVE_WAIT_MS`，后续最好接入驱动真实到位反馈。

当前 `FruitPick_PickOne()` 会临时提高 XYZ 运动速度和加速度：

```c
#define FRUIT_PICK_ARM_VEL_RPM 800u
#define FRUIT_PICK_ARM_ACC     80u
```

抓取流程结束后会恢复 `arm_motion` 默认参数。当前 `arm_motion` 默认参数也是 `800rpm / acc=80`；保留 `fruit_pick.c` 内独立宏，方便后续单独调整抓取阶段速度。如果实车出现抖动、冲击、丢步或结构晃动，优先降低 `fruit_pick.c` 中这两个抓取速度参数；如果动作稳定但仍然太慢，再逐步增加。

禁止：

1. 不要设计剪切点。
2. 不要在剪断后加入 Z 轴上抬。
3. 不要让 XYZ 带着相机扫描。

## 12. 代码修改规则

AI 每次写代码前必须：

1. 先阅读 `AGENTS.md`。
2. 用 10 行以内复述对当前硬件和目标的理解。
3. 列出准备新增或修改的文件。
4. 列出假设已经存在的底层函数。
5. 不确定的函数必须标 TODO。
6. 再给代码或开始修改。

AI 每次修改代码后必须：

1. 说明修改了哪些文件。
2. 说明如何测试。
3. 说明可能的风险。
4. 如果发现新的硬件事实或参数，要建议更新 `AGENTS.md`。
5. 不要私自改变已确认的硬件参数。

工程代码规则：

1. 尽量沿用现有模块风格和 HAL / FreeRTOS / CMSIS-RTOS v2 写法。
2. 不要重写底层驱动，只在必要处封装上层业务。
3. 新增模块要有 `.c` 和 `.h`，接口清晰，返回错误码。
4. 不要使用 `malloc`。
5. 不要引入复杂动态内存、复杂路径规划、SLAM 或多线程运动控制。
6. CubeMX 管理文件的自定义代码必须放在 `USER CODE` 块内。
7. 如果修改 Keil 工程文件，必须明确说明需要把哪些 `.c` 加入工程。

## 13. 禁止 AI 做的事情

1. 不要把所有代码写进 `StartDefaultTask`。
2. 不要创建多个运动控制任务。
3. 不要让传感器任务直接控制电机。
4. 不要使用相对位置模式封装 XYZ。
5. 不要写 `Axis_MoveRelMm`。
6. 不要让 XYZ 带相机扫描。
7. 不要假设相机装在夹爪末端。
8. 不要设计单独剪切点。
9. 不要在剪断后 Z 轴上抬。
10. 不要重写 `Emm_V5.c`。
11. 不要重写 `base_control.c`。
12. 不要使用 `malloc`。
13. 不要编造不存在的底层函数。
14. 不要一次返回多个视觉目标。
15. 不要一开始就做复杂路径规划或 SLAM。
16. 不要私自改变 X/Y/Z 电机 ID、方向和脉冲换算。

## 14. 当前调试顺序

必须按顺序调试，不要跳步：

1. 先确认 XYZ 归零和绝对位置控制。
2. 测试 `Axis_MoveToAbsMm()`。
3. 测试 `Arm_MoveToPointAbs()`。
4. 测试 `Arm_RetractSafe()`。
5. 单独测试夹爪、剪刀、分类舵机。
6. 测试树莓派 fake 通信。
7. STM32 解析 `FRUIT` 数据，只显示不动。
8. STM32 收到假坐标后只移动 XYZ，不夹不剪。
9. 测试单果抓取。
10. 测试 `WATCH -> WATCHING -> HIT -> WATCH_STOP`，只验证底盘能及时停车。
11. 测试 `HIT` 停车后 `SCAN -> FRUIT/NONE` 重拍流程。
12. 测试 `Game_CreepWatchAndPick()` 短距离巡航，例如先把累计距离设为 `60cm`。
13. 测试完整 `HIT` 停车、`SCAN` 重拍、`FruitPick_PickOne()` 夹剪投放。
14. 测试多次 HIT 打断、多次抓取后继续前进，确认累计距离到达设定值后退出。
15. 最后扩展到完整赛道和投放区。

## 15. 待确认参数区域

这些参数需要后续根据实物调试修改。未确认前，代码中必须用 TODO 或保守默认值标出。

1. `APPROACH_DISTANCE_MM`。
2. 树莓派相机到机械臂坐标映射参数。
3. 视觉 score 最低阈值。
4. 真实串口 UART 句柄名称，目前代码中树莓派串口基础封装使用 `huart2`。
5. 底盘到各棵树左/右观察点的距离、角度、循墙参数。
6. `WATCH` 模式下 Pi 侧 HIT 提前量和高效抓取窗口，当前 STM32 只做机械范围校验。
7. `GAME_CREEP_TOTAL_DISTANCE_CM`、`GAME_CREEP_SPEED_RRP`、`GAME_CREEP_STOP_SETTLE_MS` 需要根据实车停车延迟和识别速度继续调试。

## 16. 每次修改代码后的记录规则

每次 AI 修改代码后，必须在最终回复中记录：

1. 修改了哪些文件。
2. 每个文件做了什么。
3. 如何编译或测试。
4. 当前没有测试的部分和原因。
5. 可能风险。
6. 是否发现需要更新 `AGENTS.md` 的新硬件事实或参数。

不要在代码里乱加大段日志；记录规则主要体现在 AI 的回复和必要的项目文档更新中。

## 17. 给后续 AI 的当前状态提醒

1. 根目录旧 `AGENTS.md` 曾经是乱码，本文件是新的中文项目记忆。
2. 当前工程已有 `arm_motion.c/.h`，但旧接口包含相对移动，需要按绝对位置规则重构。
3. 当前工程已有 `Usart_to_Pi.c/.h`，树莓派通信可优先复用，不要重复造底层 UART 封装。
4. 当前 `freertos.c` 已加入 `Game_CreepWatchAndPick()`，用于连续巡航识别抓取；仍建议后续整理为 `Game_Init()` + `Game_Run()`。
5. 当前 `base_control.c/.h` 已加入 `Base_ForwardDistanceCmHoldYawWatchPi()`，保持航向前进时监听 Pi 的 `HIT`。
6. 当前 `pi_protocol.c/.h` 已加入 `WATCH` / `WATCHING` / `HIT` / `WATCH_STOP` 协议。
7. 当前分类逻辑：树莓派决定 `BIG` / `SMALL`，STM32 只按 `fruit.type` 选择大果/小果投放准备点和分类舵机动作；当前大小果暂存区已整体互换，大果走原小果暂存区，小果走原大果暂存区。
8. 任何运动控制修改都必须优先保护硬件安全，先小行程、低速度、单轴测试。

## 18. 注释语言规则

1. 新增或修改本项目业务代码时，头文件中的接口说明、结构体字段说明、返回值说明优先使用中文注释。
2. 英文缩写、函数名、协议字段名可以保留英文，但解释说明要用中文，方便现场调试人员阅读。
3. 不确定的硬件参数、舵机编号、角度、机械点位仍必须用 `TODO` 标出，不要写成已确认事实。
