# STM32 与树莓派通信说明文档

本文档说明当前自动采摘机器人项目中 STM32 下位机与树莓派视觉程序之间的串口通信协议、状态流程、字段含义、错误处理和调试方法。

适用代码：

- STM32 协议层：`Core/Src/pi_protocol.c`、`Core/Inc/pi_protocol.h`
- STM32 底盘监听：`Core/Src/base_control.c`
- STM32 比赛流程：`Core/Src/game_task.c`
- STM32 串口基础层：`Core/Src/Usart_to_Pi.c`、`Core/Inc/Usart_to_Pi.h`
- 树莓派视觉脚本：`树莓派视觉代码/v2.4_text.py`

## 1. 通信目标

通信链路用于让 STM32 和树莓派在自动采摘流程中分工协作。

STM32 负责：

- 发送 `PING` 确认树莓派协议可用。
- 在底盘慢速巡航时发送 `WATCH`，让树莓派持续识别苹果。
- 巡航过程中轮询 `HIT`，收到合法 `HIT` 后立即停车。
- 停车后发送 `WATCH_STOP` 结束连续识别。
- 停稳后发送 `SCAN`，请求树莓派重新取帧并返回最终抓取目标。
- 收到 `FRUIT` 后控制 XYZ、夹爪、剪刀和分类舵机完成抓取。
- 收到 `NONE`、超时或非法目标时跳过本次抓取并继续巡航。

树莓派负责：

- 接收 STM32 的 `PING`、`WATCH`、`WATCH_STOP`、`SCAN`、`SCAN_CANCEL`。
- 使用 RealSense 深度相机和 ONNX YOLO 检测苹果。
- 根据深度估算苹果真实直径，判断 `BIG` 或 `SMALL`。
- 将相机坐标映射为机械臂绝对坐标 `x/y/z`，单位 mm。
- `WATCH` 模式下仅在目标进入可抓窗口且稳定时发送 `HIT`。
- `SCAN` 模式下重新取帧，返回最终 `FRUIT` 或 `NONE`。

核心原则：

- `HIT` 只用于通知 STM32 停车，不作为最终抓取坐标。
- 最终抓取只使用停车稳定后 `SCAN` 返回的 `FRUIT`。
- `type` 由树莓派决定，STM32 不根据 `diameter` 二次判断大小果。
- `x/y/z` 是机械臂绝对坐标，不是相机坐标。

## 2. 物理链路与基础格式

当前 STM32 侧使用 `huart2` 与树莓派通信。

| 项目 | 当前实现 |
|---|---|
| STM32 串口 | `USART2 / huart2` |
| STM32 基础封装 | `PiUart2_Init()`、`PiUart2_StartRx()`、`PiUart2_GetFrame()`、`PiUart2_Send()` |
| 接收方式 | `HAL_UARTEx_ReceiveToIdle_DMA()` + 环形缓冲 |
| 发送方式 | `HAL_UART_Transmit_DMA()` |
| 树莓派串口设备 | 自动扫描 CH340，VID `0x1A86`，PID `0x7523` |
| 波特率 | `115200` |
| 数据格式 | `8N1` |
| 消息编码 | ASCII / UTF-8 兼容文本 |
| 结束符 | 每条控制消息以 `\n` 结束，STM32 也兼容 `\r` |
| CRC | 当前无 CRC |
| STM32 协议行缓存 | `128` 字节 |
| STM32 UART 收发缓存 | `256` 字节 |

协议消息采用逗号分隔文本：

```text
CMD,field1,field2,...\n
```

约定：

- 命令字使用大写英文。
- 字段之间用英文逗号 `,` 分隔。
- 一条协议消息必须在一行内完成。
- 坐标和直径单位均为 mm。
- score 使用整数百分制，范围 `0~100`。
- STM32 解析整数使用 `strtol()`，支持负号；负坐标会被判定为越界。

## 3. 序号 seq 机制

`seq` 用于请求和响应配对。

STM32 规则：

- `Pi_Ping(seq, timeout_ms)` 的 `seq` 由调用者传入。
- `WATCH`、`SCAN` 的 `seq` 由 `pi_protocol.c` 内部递增生成。
- 自动递增初始值为 `2`。
- `seq` 为 `uint16_t` 范围，遇到 `0` 会跳回 `2`。
- STM32 等待响应时只接受相同 `seq` 的有效回复。
- 旧 `seq` 或无关行一般会被忽略，继续等待当前请求。

树莓派规则：

- 收到请求后，回复必须带回相同 `seq`。
- `WATCH` 后持续发送的 `HIT` 必须使用该次 `WATCH` 的 `seq`。
- `SCAN` 后返回的 `FRUIT`、`NONE`、`ERR` 必须使用该次 `SCAN` 的 `seq`。

示例：

```text
STM32 -> PI: WATCH,2,0,L\n
PI    -> STM32: WATCHING,2\n
PI    -> STM32: HIT,2,BIG,128,188,85,83,93,2,GOOD\n
STM32 -> PI: WATCH_STOP,2\n
STM32 -> PI: SCAN,3,0,L\n
PI    -> STM32: FRUIT,3,BIG,126,190,86,83,94\n
```

## 4. STM32 发给树莓派的消息

### 4.1 PING

用途：确认树莓派程序已经启动并且协议可用。

格式：

```text
PING,seq\n
```

字段：

| 字段 | 含义 |
|---|---|
| `seq` | 请求序号，`0~65535` |

期望回复：

```text
PONG,seq\n
```

当前使用位置：

- `Game_WaitPiReadyAndUserStart()` 循环发送 `PING`。
- 收到 `PONG` 后，OLED 状态切到 `PI READY K1`，等待 KEY1 消抖按下。

### 4.2 WATCH

用途：启动树莓派连续识别。STM32 一边慢速巡航，一边等待树莓派主动报告可抓目标。

格式：

```text
WATCH,seq,tree_id,view_id\n
```

字段：

| 字段 | 含义 |
|---|---|
| `seq` | 本次 WATCH 序号，由 STM32 自动生成 |
| `tree_id` | 当前果树编号，树莓派当前代码只保存，主要用于后续扩展和调试 |
| `view_id` | 观察方向，`L` 表示左侧，`R` 表示右侧 |

期望立即回复：

```text
WATCHING,seq\n
```

后续异步回复：

```text
HIT,seq,type,x,y,z,diameter,score,stable,zone\n
ERR,seq,error_code\n
```

当前使用位置：

- `Base_ForwardDistanceCmHoldYawWatchPi()` 进入巡航前调用 `Pi_StartWatch()`。
- `Pi_StartWatch()` 发送 `WATCH` 后等待 `WATCHING`，超时为 `1000ms`。

### 4.3 WATCH_STOP

用途：结束当前 `WATCH` 模式。通常在收到合法 `HIT` 停车后发送，也会在正常巡航结束、超时或异常退出前发送。

格式：

```text
WATCH_STOP,seq\n
```

字段：

| 字段 | 含义 |
|---|---|
| `seq` | 要停止的 WATCH 序号 |

回复：

- 当前树莓派代码不回复 `WATCH_STOP`。
- STM32 发送后不等待确认。

树莓派行为：

- 如果当前处于 `watch_mode` 且 `watch_seq` 匹配，则退出 WATCH。
- 清空 `watch_hit_sent`、`watch_candidate`、`watch_stable_count`。

### 4.4 SCAN

用途：停车稳定后重新取帧，获得最终抓取目标。

格式：

```text
SCAN,seq,tree_id,view_id\n
```

字段：

| 字段 | 含义 |
|---|---|
| `seq` | 本次 SCAN 序号，由 STM32 自动生成 |
| `tree_id` | 当前果树编号 |
| `view_id` | 观察方向，`L` 或 `R` |

期望回复：

```text
FRUIT,seq,type,x,y,z,diameter,score\n
NONE,seq\n
NONE,seq,reason\n
ERR,seq,error_code\n
```

当前使用位置：

- `Game_CreepWatchAndPick()` 收到 `HIT` 并停稳后调用 `Pi_RequestBestFruitWithSeq()`。
- STM32 侧 `SCAN` 等待超时为 `10000ms`。
- 树莓派脚本内部 `SCAN_TIMEOUT_S` 当前为 `3.0s`，超时后不回包，由 STM32 侧超时处理。

注意：

- `SCAN` 返回 `FRUIT` 才能进入 `FruitPick_PickOne()`。
- `SCAN` 返回 `NONE` 时，本次不抓取，继续剩余距离巡航。
- `SCAN` 超时时，STM32 会发送 `SCAN_CANCEL`，然后继续巡航。

### 4.5 SCAN_CANCEL

用途：STM32 等待 `SCAN` 超时后，通知树莓派取消指定 `SCAN` 结果，避免树莓派晚发的旧 `FRUIT` 影响后续流程。

格式：

```text
SCAN_CANCEL,seq\n
```

字段：

| 字段 | 含义 |
|---|---|
| `seq` | 要取消的 SCAN 序号 |

回复：

- 当前树莓派代码不回复。

树莓派行为：

- 将该 `seq` 加入 `canceled_scan_seqs`。
- 如果当前正在处理同一个 `scan_seq`，立即结束本次 SCAN。
- 如果结果稍后才算出，会丢弃该结果，不发送 `FRUIT/NONE/ERR`。

## 5. 树莓派发给 STM32 的消息

### 5.1 PONG

用途：响应 `PING`。

格式：

```text
PONG,seq\n
```

STM32 处理：

- `seq` 匹配时返回 `PI_OK`。
- `seq` 不匹配或格式错误时返回 `PI_ERR_PARSE`，等待层通常会忽略旧 `seq` 行。

### 5.2 WATCHING

用途：确认树莓派已进入 `WATCH` 连续识别模式。

格式：

```text
WATCHING,seq\n
```

STM32 处理：

- `Pi_StartWatch()` 收到匹配 `WATCHING` 后返回 `PI_OK`。
- 如果 `1000ms` 内没有收到，巡航监听流程不会启动，函数返回通信错误。

### 5.3 HIT

用途：`WATCH` 模式下通知 STM32 目标已经进入可抓窗口，需要立即停车。

当前实际格式：

```text
HIT,seq,type,x,y,z,diameter,score,stable,zone\n
```

字段：

| 字段 | 含义 | 合法值或范围 |
|---|---|---|
| `seq` | 对应 WATCH 的序号 | `0~65535` |
| `type` | 果子类型 | `BIG` 或 `SMALL` |
| `x` | 机械臂 X 绝对坐标，mm | `0~220` |
| `y` | 机械臂 Y 绝对坐标，mm | `40~480` |
| `z` | 机械臂 Z 绝对坐标，mm | `0~400` |
| `diameter` | 估算真实直径，mm | STM32 当前接受 `40~100` |
| `score` | 视觉置信度，整数百分制 | `0~100` |
| `stable` | 连续稳定帧数 | STM32 当前要求 `>= 2` |
| `zone` | HIT 窗口状态 | STM32 当前要求 `GOOD` |

树莓派发送条件：

- `type` 必须是 `BIG` 或 `SMALL`。
- 映射后的机械臂坐标必须在可达范围内。
- 当前脚本 `WATCH_SEND_ANY_REACHABLE_ARM = True` 时，只要坐标可达即可进入候选；否则还会检查 score、有效深度比例和边缘过滤。
- 当前候选与上一候选在类型、深度、直径、坐标上连续稳定。
- `watch_stable_count >= WATCH_STABLE_FRAMES`，当前为 `2`。
- `get_hit_zone(x)` 必须返回 `GOOD`。

STM32 处理：

- `Base_ForwardDistanceCmHoldYawWatchPi()` 每个控制周期调用 `Pi_PollWatchHit(watch_seq, &hit, 20u)`。
- 只有 `Pi_PollWatchHit()` 返回 `PI_OK` 才立即停车。
- `HIT` 坐标越界返回 `PI_ERR_OUT_OF_RANGE`，底盘不会停车，会继续巡航。
- 非超时、非越界的通信错误会停车并退出，避免异常状态下继续行驶。
- 收到合法 `HIT` 后，底盘停止，发送 `WATCH_STOP`，返回 `1` 给上层。

重要说明：

- STM32 不使用 `HIT` 坐标抓取。
- 合法 `HIT` 只代表“值得停车重拍”。
- 如果树莓派仍按旧格式发送 `HIT,seq,type,x,y,z,diameter,score`，当前 STM32 会解析失败，不会作为有效 HIT。

### 5.4 FRUIT

用途：`SCAN` 模式下返回最终抓取目标。

格式：

```text
FRUIT,seq,type,x,y,z,diameter,score\n
```

字段：

| 字段 | 含义 | 合法值或范围 |
|---|---|---|
| `seq` | 对应 SCAN 的序号 | `0~65535` |
| `type` | 果子类型 | `BIG` 或 `SMALL` |
| `x` | 机械臂 X 绝对坐标，mm | `0~220` |
| `y` | 机械臂 Y 绝对坐标，mm | `40~480` |
| `z` | 机械臂 Z 绝对坐标，mm | `0~400` |
| `diameter` | 估算真实直径，mm | STM32 当前接受 `40~100` |
| `score` | 视觉置信度，整数百分制 | `0~100` |

STM32 处理：

- `Pi_RequestBestFruitWithSeq()` 收到合法 `FRUIT` 后返回 `PI_OK`。
- `Game_CreepWatchAndPick()` 调用 `FruitPick_PickOne(&fruit)`。
- 抓取类型完全按 `type` 执行，大果和小果投放点不同。

坐标校验：

- 任意坐标为负数，返回 `PI_ERR_OUT_OF_RANGE`。
- `y < ARM_MOTION_Y_PRE_EXTEND_MM`，当前安全线为 `40mm`，返回 `PI_ERR_OUT_OF_RANGE`。
- `Arm_IsPointReachable()` 判定不可达，返回 `PI_ERR_OUT_OF_RANGE`。
- `diameter` 不在 `40~100`，返回 `PI_ERR_PARSE`。
- `score` 不在 `0~100`，返回 `PI_ERR_PARSE`。

### 5.5 NONE

用途：`SCAN` 模式下表示没有合适目标。

基础格式：

```text
NONE,seq\n
```

扩展格式：

```text
NONE,seq,reason\n
```

STM32 当前兼容的 `reason`：

| reason | STM32 枚举 | 含义 |
|---|---|---|
| `NO_TARGET` | `PI_NONE_REASON_NO_TARGET` | 未检测到合适目标 |
| `OUT_OF_RANGE` | `PI_NONE_REASON_OUT_OF_RANGE` | 坐标超出机械臂可达范围 |
| `UNKNOWN_SIZE` | `PI_NONE_REASON_UNKNOWN_SIZE` | 大小无法判定 |
| `LOW_SCORE` | `PI_NONE_REASON_LOW_SCORE` | 置信度过低 |
| `OCCLUDED` | `PI_NONE_REASON_OCCLUDED` | 遮挡严重 |

当前树莓派脚本实际发送：

```text
NONE,seq\n
```

STM32 处理：

- `Pi_ParseLine()` 返回 `PI_ERR_NONE`。
- `Pi_GetLastNoneReason()` 可读取最近一次 NONE 原因。
- 如果没有 reason，STM32 记录为 `PI_NONE_REASON_NO_TARGET`。
- `Game_CreepWatchAndPick()` 不抓取，继续按剩余距离巡航。

### 5.6 ERR

用途：树莓派报告当前请求处理失败。

格式：

```text
ERR,seq,error_code\n
```

字段：

| 字段 | 含义 |
|---|---|
| `seq` | 对应请求序号 |
| `error_code` | 树莓派错误码字符串，例如 `FRAME_TIMEOUT`、`DEPTH_INVALID` 等 |

STM32 处理：

- `seq` 匹配时返回 `PI_ERR_REMOTE`。
- 当前 STM32 只识别远端错误，不解析具体 `error_code` 内容。
- `WATCH` 中收到远端错误会停止底盘并返回错误。
- `SCAN` 中收到远端错误会使 `Game_CreepWatchAndPick()` 返回错误。

### 5.7 CAM 调试行

当前树莓派脚本在空闲预览时可能发送调试行：

```text
CAM,cam=(x,y,z)\n
```

说明：

- 该行不是正式控制协议。
- STM32 `PiProtocol_WaitParsedLine()` 会把未知前缀视为无关行，继续等待当前 `seq` 的正式回复。
- `Pi_PollWatchHit()` 在等待 HIT 时收到非 `HIT/ERR` 行，会当作本轮轮询超时处理。

## 6. 数据字段详细说明

### 6.1 view_id

| 协议字符 | STM32 枚举 |
|---|---|
| `L` | `TREE_VIEW_LEFT` |
| `R` | `TREE_VIEW_RIGHT` |

当前项目记忆中左右观察点功能已删除，但协议仍保留 `view_id` 字段，方便后续根据树、方向或赛道阶段做视觉参数区分。

### 6.2 type

| 协议文本 | STM32 类型 | 行为 |
|---|---|---|
| `BIG` | `FRUIT_BIG` / `FRUIT_TYPE_BIG` | 大果投放流程 |
| `SMALL` | `FRUIT_SMALL` / `FRUIT_TYPE_SMALL` | 小果投放流程 |

注意：

- STM32 当前只接受 `BIG` 和 `SMALL`。
- `UNKNOWN` 不应发送给 STM32；如果树莓派无法判断大小，应返回 `NONE` 或 `ERR`。
- 当前 `v2.4_text.py` 中 `BIG_MIN_MM = 62.0`、`SMALL_MAX_MM = 62.0`，与项目记忆中“大果建议 >=70、小果 <=62、中间 UNKNOWN”不完全一致。实车调试时如果需要恢复中间 UNKNOWN 区间，应同步修改树莓派脚本和本文档。

### 6.3 x/y/z

`x/y/z` 是树莓派映射后的机械臂绝对坐标，单位 mm。

当前机械臂范围：

| 轴 | 最小值 | 最大值 |
|---|---:|---:|
| X | `0` | `220` |
| Y | `0` | `480` |
| Z | `0` | `400` |

普通抓取和巡航流程中，Y 轴安全线为：

```text
Y >= 40mm
```

因此通信层实际接受范围：

| 轴 | 通信层抓取接受范围 |
|---|---:|
| X | `0~220` |
| Y | `40~480` |
| Z | `0~400` |

### 6.4 diameter

`diameter` 是树莓派根据 bbox 像素尺寸、深度值和相机内参估算出的真实直径，单位 mm。

STM32 当前只做合理性检查：

```text
40 <= diameter <= 100
```

STM32 不使用 `diameter` 重新判断 `BIG/SMALL`。

### 6.5 score

`score` 是树莓派返回的整数百分制置信度。

STM32 当前只做范围检查：

```text
0 <= score <= 100
```

是否抓取、大小分类和坐标映射主要由树莓派完成。

### 6.6 stable

`stable` 只出现在当前实现的 `HIT` 消息中，表示 WATCH 候选目标连续稳定帧数。

STM32 当前要求：

```text
stable >= 2
```

树莓派当前配置：

```python
WATCH_STABLE_FRAMES = 2
```

### 6.7 zone

`zone` 只出现在当前实现的 `HIT` 消息中，表示目标在 HIT 窗口中的位置状态。

当前合法值：

| zone | 含义 | STM32 行为 |
|---|---|---|
| `EARLY` | 目标还未进入理想窗口 | 不作为有效 HIT |
| `GOOD` | 目标在可停车窗口内 | 可作为有效 HIT |
| `LATE` | 目标已经越过理想窗口 | 不作为有效 HIT |

STM32 当前要求：

```text
zone == GOOD
```

树莓派当前 `get_hit_zone(arm_x)` 使用 X 坐标判断：

```text
HIT_ZONE_X_MIN <= arm_x <= HIT_ZONE_X_MAX => GOOD
```

当前脚本中 HIT 窗口为：

```text
0 <= arm_x <= 220
```

## 7. 典型完整流程

### 7.1 上电等待树莓派

```text
STM32 -> PI: PING,1\n
PI    -> STM32: PONG,1\n
```

STM32 行为：

1. `Game_WaitPiReadyAndUserStart()` 循环发送 `PING`。
2. 收到 `PONG` 后认为 Pi 协议就绪。
3. 等待 KEY1 消抖按下后进入比赛主流程。

### 7.2 巡航识别并停车

```text
STM32 -> PI: WATCH,2,0,L\n
PI    -> STM32: WATCHING,2\n
PI    -> STM32: HIT,2,BIG,128,188,85,83,93,2,GOOD\n
STM32 -> PI: WATCH_STOP,2\n
```

STM32 行为：

1. `Base_ForwardDistanceCmHoldYawWatchPi()` 发送 `WATCH`。
2. 底盘保持目标 yaw 慢速前进。
3. 每 `20ms` 左右轮询一次 `Pi_PollWatchHit()`。
4. 收到合法 `HIT` 后立刻调用底盘停止。
5. 估算并返回本段已前进距离。
6. 发送 `WATCH_STOP`。
7. 返回 `1` 给 `Game_CreepWatchAndPick()`，表示“因 HIT 停车”。

### 7.3 停车后重拍并抓取

```text
STM32 -> PI: SCAN,3,0,L\n
PI    -> STM32: FRUIT,3,BIG,126,190,86,83,94\n
```

STM32 行为：

1. 停车后 `osDelay(GAME_CREEP_STOP_SETTLE_MS)`，当前为 `800ms`。
2. 发送 `SCAN`，等待最终目标。
3. 收到合法 `FRUIT` 后调用 `FruitPick_PickOne()`。
4. 完成抓取、剪切、分类投放和安全回收。
5. 额外等待 `GAME_CREEP_RESUME_DELAY_MS`，当前为 `3500ms`。
6. 继续剩余距离巡航。

### 7.4 停车后没有目标

```text
STM32 -> PI: SCAN,4,0,L\n
PI    -> STM32: NONE,4\n
```

STM32 行为：

1. `Pi_RequestBestFruitWithSeq()` 返回 `PI_ERR_NONE`。
2. `Game_CreepWatchAndPick()` 记录 `g_game_creep_last_none_reason`。
3. 本次不抓取，继续剩余距离巡航。

### 7.5 SCAN 超时并取消

```text
STM32 -> PI: SCAN,5,0,L\n
... 10000ms 内没有有效回复 ...
STM32 -> PI: SCAN_CANCEL,5\n
```

STM32 行为：

1. `Pi_RequestBestFruitWithSeq()` 返回 `PI_ERR_TIMEOUT`。
2. `g_game_creep_scan_timeout_count++`。
3. 发送 `SCAN_CANCEL,5`。
4. 本次不抓取，继续巡航。

树莓派行为：

- 如果稍后才算出 `SCAN,5` 的结果，应丢弃，不再发送。

## 8. STM32 协议返回值

`pi_protocol.h` 中定义：

| 返回值 | 含义 |
|---:|---|
| `PI_OK = 0` | 成功 |
| `PI_ERR_TIMEOUT = -1` | 等待超时，或 WATCH 轮询本轮没有 HIT |
| `PI_ERR_PARSE = -2` | 收到当前 seq 的行，但格式非法 |
| `PI_ERR_NONE = -3` | SCAN 返回 NONE |
| `PI_ERR_REMOTE = -4` | 树莓派返回 ERR |
| `PI_ERR_OUT_OF_RANGE = -5` | 坐标越界或机械臂不可达 |
| `PI_ERR_PARAM = -6` | 本地参数错误，例如空指针或 timeout 为 0 |

## 9. 超时与周期

| 位置 | 当前值 | 说明 |
|---|---:|---|
| `Pi_StartWatch()` 等待 `WATCHING` | `1000ms` | 底盘巡航前确认 Pi 进入 WATCH |
| `Pi_PollWatchHit()` 单次轮询 | `20ms` | 底盘控制循环内短等待 |
| `Game_CreepWatchAndPick()` 停车稳定等待 | `800ms` | HIT 停车后再 SCAN |
| `Pi_RequestBestFruitWithSeq()` 等待 `SCAN` 结果 | `10000ms` | STM32 等待最终 FRUIT/NONE/ERR |
| 树莓派 `SCAN_TIMEOUT_S` | `3.0s` | Pi 内部处理超时时间，超时后当前不回包 |
| 抓取后恢复巡航等待 | `3500ms` | 避免 XYZ 刚回收就继续巡航 |
| 树莓派 `WATCH_MIN_INTERVAL_S` | `0.1s` | WATCH 识别刷新间隔 |
| 树莓派 `SERIAL_READ_TIMEOUT_S` | `0.02s` | 主循环串口 readline 超时 |

调试建议：

- 如果树莓派推理变慢，应优先保证 STM32 `SCAN` 等待时间不低于 `10000ms`。
- 如果 `SCAN` 经常超时且树莓派晚发结果，应检查 `SCAN_CANCEL` 是否被正确处理。
- 如果底盘停车偏晚，应调小树莓派 HIT 提前量或优化 `zone` 窗口，而不是直接用 `HIT` 坐标抓取。

## 10. 树莓派视觉处理与发包逻辑

树莓派每次识别的大致流程：

1. 从 RealSense 后台线程获取最新 RGB 和 Depth。
2. YOLO 检测苹果 bbox。
3. 在 bbox 内采样深度，估算中心 3D 相机坐标。
4. 根据 bbox 像素尺寸、深度和相机内参估算真实直径。
5. `classify_apple(diameter_mm)` 判断 `BIG/SMALL/UNKNOWN`。
6. `camera_to_arm()` 将相机坐标映射为机械臂绝对坐标。
7. `is_arm_point_reachable()` 检查机械臂坐标范围。
8. 生成 `result`，包含 `type/x/y/z/diameter/score/bbox/valid_ratio/depth_mm`。

当前脚本关键参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `CONF_THRESHOLD` | `0.5` | YOLO 检测阈值 |
| `BIG_MIN_MM` | `62.0` | 大果判断下限 |
| `SMALL_MAX_MM` | `62.0` | 小果判断上限 |
| `WATCH_STABLE_FRAMES` | `2` | HIT 稳定帧数 |
| `WATCH_SCORE_MIN` | `85` | WATCH 候选最低分，当前在 `WATCH_SEND_ANY_REACHABLE_ARM=True` 时不强制使用 |
| `WATCH_SEND_ANY_REACHABLE_ARM` | `True` | 可达即允许 WATCH 候选 |
| `POS_JUMP_MAX_MM` | `25` | 坐标稳定性跳变阈值 |
| `DIAMETER_JUMP_MAX_MM` | `12` | 直径稳定性跳变阈值 |
| `DEPTH_JUMP_MAX_MM` | `80` | 深度稳定性跳变阈值 |
| `ARM_X_MIN/MAX` | `0/220` | Pi 侧 X 可达范围 |
| `ARM_Y_MIN/MAX` | `40/480` | Pi 侧 Y 可达范围 |
| `ARM_Z_MIN/MAX` | `0/400` | Pi 侧 Z 可达范围 |

## 11. STM32 端状态流程

### 11.1 `PiProtocol_WaitLine()`

功能：

- 从 `PiUart2_GetFrame()` 读取 UART DMA 环形缓冲中的字节。
- 按 `\n` 或 `\r` 拼成完整文本行。
- 去掉行尾空格和 tab。
- 如果行累计超过 `PI_PROTOCOL_LINE_BUF_LEN - 1`，返回 `PI_ERR_PARSE`。
- 所有等待都有 timeout。

### 11.2 `Pi_ParseLine()`

支持解析：

- `PONG,seq`
- `WATCHING,seq`
- `HIT,seq,type,x,y,z,diameter,score,stable,zone`
- `FRUIT,seq,type,x,y,z,diameter,score`
- `NONE,seq`
- `NONE,seq,reason`
- `ERR,seq,error_code`

不支持解析：

- 多目标列表。
- JSON。
- 不带 `seq` 的正式控制结果。
- `UNKNOWN` 类型目标。

### 11.3 `PiProtocol_ShouldIgnoreParseError()`

用途：

- 收到旧 `seq` 的 `PONG/WATCHING/NONE/ERR/FRUIT/HIT` 时忽略。
- 收到未知前缀，例如 `CAM,...` 时忽略。
- 如果当前 `seq` 对上但字段格式错误，不忽略，返回 `PI_ERR_PARSE`。

### 11.4 `Base_ForwardDistanceCmHoldYawWatchPi()`

流程：

1. 发送 `WATCH` 并等待 `WATCHING`。
2. 底盘保持 yaw 前进。
3. 循环调用 `Pi_PollWatchHit()`。
4. `PI_OK`：停车，发送 `WATCH_STOP`，返回 `1`。
5. `PI_ERR_TIMEOUT`：本轮没有 HIT，继续前进。
6. `PI_ERR_OUT_OF_RANGE`：HIT 坐标越界，不停车，继续前进。
7. 其他错误：停车，发送 `WATCH_STOP`，返回错误码。
8. 到达目标距离仍无 HIT：停车，发送 `WATCH_STOP`，返回 `0`。

### 11.5 `Game_CreepWatchAndPick()`

流程：

1. 计算剩余巡航距离。
2. 调用 `Base_ForwardDistanceCmHoldYawWatchPi()`。
3. 返回 `0`：剩余距离走完，函数结束。
4. 返回 `1`：因 HIT 停车，进入 SCAN。
5. 停稳等待 `800ms`。
6. 发送 `SCAN`，等待 `FRUIT/NONE/ERR`。
7. `FRUIT`：调用 `FruitPick_PickOne()`。
8. `NONE`：不抓取，继续剩余距离。
9. `SCAN` 超时：发送 `SCAN_CANCEL`，继续剩余距离。
10. 其他错误：向上返回错误码。

## 12. 异常处理策略

### 12.1 旧包或无关包

场景：

- STM32 正在等 `SCAN,3`，串口里残留 `PONG,1`。
- 树莓派发送了调试行 `CAM,...`。

处理：

- STM32 忽略该行，继续等待当前 `seq`。

### 12.2 当前 seq 格式错误

场景：

```text
FRUIT,3,BIG,128,188,85,83\n
```

缺少 `score` 字段。

处理：

- STM32 返回 `PI_ERR_PARSE`。
- 上层视为通信错误，一般会停止或退出当前流程。

### 12.3 HIT 越界

场景：

```text
HIT,2,BIG,128,20,85,83,93,2,GOOD\n
```

Y 小于 `40mm`。

处理：

- `PiProtocol_CheckFruitRange()` 返回 `PI_ERR_OUT_OF_RANGE`。
- `Base_ForwardDistanceCmHoldYawWatchPi()` 对越界 HIT 不停车，继续巡航。

### 12.4 FRUIT 越界

场景：

```text
FRUIT,3,BIG,128,20,85,83,93\n
```

处理：

- `Pi_RequestBestFruitWithSeq()` 返回 `PI_ERR_OUT_OF_RANGE`。
- `Game_CreepWatchAndPick()` 当前会返回该错误，不进入抓取。

### 12.5 树莓派远端错误

场景：

```text
ERR,3,DEPTH_INVALID\n
```

处理：

- STM32 返回 `PI_ERR_REMOTE`。
- 当前 STM32 不细分 `error_code`，调试时需要查看树莓派终端日志。

### 12.6 树莓派不回包

场景：

- `SCAN` 后树莓派内部 `SCAN_TIMEOUT_S` 到期，当前代码直接结束本次 SCAN，不发送 `NONE`。

处理：

- STM32 等到 `10000ms` 后返回 `PI_ERR_TIMEOUT`。
- 上层发送 `SCAN_CANCEL` 并继续巡航。

## 13. 调试方法

### 13.1 通信连通性测试

步骤：

1. 树莓派运行 `v2.4_text.py`。
2. 确认终端打印 CH340 端口已打开，波特率为 `115200`。
3. STM32 调用 `Game_WaitPiReadyAndUserStart()`。
4. OLED 应显示 `PI WAIT`，收到 `PONG` 后显示 `PI READY K1`。
5. 按 KEY1 后进入后续流程。

预期串口：

```text
PING,1
PONG,1
```

### 13.2 fake FRUIT 测试

可以用串口工具模拟树莓派，按当前 `seq` 回复：

```text
FRUIT,3,BIG,128,188,85,83,93\n
```

注意：

- `seq` 必须与 STM32 当前等待的 `SCAN` 序号一致。
- `y` 必须大于等于 `40`。
- 坐标必须在机械臂范围内。

### 13.3 WATCH/HIT 停车测试

模拟流程：

```text
STM32 -> WATCH,2,0,L
PC    -> WATCHING,2
PC    -> HIT,2,BIG,128,188,85,83,93,2,GOOD
STM32 -> WATCH_STOP,2
```

预期：

- 底盘收到合法 `HIT` 后立即停车。
- 上层随后进入 `SCAN`。

常见错误：

- 如果只发旧格式 `HIT,2,BIG,128,188,85,83,93`，当前 STM32 不会认为合法。
- 如果 `stable=1`，当前 STM32 不会认为合法。
- 如果 `zone=EARLY` 或 `LATE`，当前 STM32 不会认为合法。

### 13.4 SCAN_CANCEL 测试

模拟树莓派长时间不回复 `SCAN`。

预期：

```text
STM32 -> SCAN,5,0,L
... 约 10000ms 后 ...
STM32 -> SCAN_CANCEL,5
```

树莓派应丢弃 `seq=5` 的后续结果。

## 14. 常见问题排查

### 14.1 STM32 一直显示 PI WAIT

可能原因：

- 树莓派脚本未运行。
- CH340 未连接或权限不足。
- 波特率不是 `115200`。
- STM32 `huart2` 接线错误。
- 树莓派收到 `PING` 但没有以 `PONG,seq\n` 回复。

### 14.2 WATCH 后底盘不动

可能原因：

- `Pi_StartWatch()` 未收到 `WATCHING`。
- 树莓派脚本未进入 `watch_mode`。
- `view_id` 字段异常，不是 `L/R`。

### 14.3 看到苹果但不停车

可能原因：

- 树莓派没有发送 `HIT`。
- `HIT` 坐标越界，STM32 忽略越界 HIT。
- `stable < 2`。
- `zone != GOOD`。
- 树莓派发的是旧格式 HIT，缺少 `stable,zone`。
- `seq` 不匹配，STM32 把它当旧包忽略。

### 14.4 停车后不抓取

可能原因：

- `SCAN` 返回 `NONE`。
- `SCAN` 超时并触发 `SCAN_CANCEL`。
- `FRUIT` 坐标越界。
- `FRUIT` 格式错误。
- `FruitPick_PickOne()` 返回抓取动作错误。

### 14.5 树莓派晚发 FRUIT 导致错抓风险

当前规避方式：

- STM32 超时后发送 `SCAN_CANCEL,seq`。
- 树莓派收到取消后丢弃该 `seq` 的结果。
- STM32 等待层也会忽略旧 `seq` 的 `FRUIT`。

## 15. 协议版本注意事项

当前代码相对基础项目记忆有以下扩展：

1. `HIT` 当前实际格式包含 `stable,zone` 两个额外字段。
2. STM32 只接受 `stable >= 2` 且 `zone == GOOD` 的 HIT。
3. STM32 支持 `NONE,seq,reason`，但当前树莓派脚本主要发送 `NONE,seq`。
4. STM32 支持 `SCAN_CANCEL,seq`，用于取消晚到的 SCAN 结果。
5. 树莓派可能发送 `CAM,cam=(...)` 调试行，STM32 会忽略。
6. 当前树莓派大小果阈值为 `62/62`，与项目记忆中的 `>=70` 和 `<=62` 建议不完全一致。

后续如果修改任一协议字段，必须同步修改：

- `Core/Src/pi_protocol.c`
- `Core/Inc/pi_protocol.h`
- `树莓派视觉代码/v2.4_text.py`
- 本文档
- 必要时同步更新 `AGENTS.md`

