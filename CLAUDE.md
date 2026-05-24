# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述 (Project Overview)

STM32F407VET6 嵌入式微控制器项目，运行FreeRTOS操作系统，支持LED控制、按键输入、OLED显示、舵机控制、激光测距和SBUS遥控接收。

**Hardware Platform:** STM32F407VET6 
**RTOS:** FreeRTOS
**Build System:** Keil µVision (MDK-ARM v5)

### 核心功能模块
- **激光测距系统**: 4路LD14激光雷达（3路STM32 UART + 1路SC16IS752通道A）
- **遥控接收**: SBUS协议（SC16IS752通道B）
- **显示系统**: OLED屏幕（I2C接口）
- **舵机控制**: 多路PWM舵机控制
- **人机交互**: LED指示灯和按键输入

## 硬件配置 (Hardware Configuration)

### GPIO映射
- **LED输出（高电平点亮）:**
  - PD13: LED2
  - PD14: LED3
  - PE2: LED1

- **按键输入（低电平=按下）:**
  - PE3: KEY1
  - PE4: KEY2
  - PE5: KEY3
  - PE6: KEY4

### 串口配置
- **UART4 (PC10/PC11)**: 激光雷达1，230400bps，8N1，DMA循环接收
- **UART5 (PC12/PD2)**: 激光雷达2，230400bps，8N1，DMA循环接收
- **USART6 (PC6/PC7)**: 激光雷达3，230400bps，8N1，DMA循环接收

### SPI配置
- **SPI1 (PA5/PA6/PA7)**: SC16IS752双串口扩展芯片
  - CS: PB12
  - IRQ#: PB14 (EXTI下降沿触发)
  - 通道A: 激光雷达4，230400bps，8N1
  - 通道B: SBUS接收，100000bps，8E2

### I2C配置
- **I2C1 (PB8/PB9)**: OLED显示屏，DMA传输

## 项目结构 (Project Structure)

```
v1.0/
├── Core/
│   ├── Inc/                       # 头文件目录
│   │   ├── main.h                 # 主头文件
│   │   ├── gpio.h                 # GPIO配置（CubeMX生成）
│   │   ├── dma.h / i2c.h / spi.h / tim.h / usart.h  # 外设配置（CubeMX生成）
│   │   ├── stm32f4xx_it.h         # 中断处理声明
│   │   ├── stm32f4xx_hal_conf.h   # HAL库配置
│   │   ├── FreeRTOSConfig.h       # FreeRTOS配置
│   │   ├── led.h / button.h       # 基础外设模块
│   │   ├── oled.h                 # OLED显示驱动
│   │   ├── servo.h                # 舵机控制（8路PWM）
│   │   ├── sc16is752.h            # SPI转双串口芯片驱动
│   │   ├── sc16_tasks.h           # SC16IS752任务层
│   │   ├── ld14.h                 # LD14激光雷达协议解析
│   │   ├── lidar_manager.h        # 4路激光雷达统一管理
│   │   ├── sbus.h                 # SBUS遥控协议解析
│   │   └── Emm_V5.h               # 步进电机驱动（预留）
│   └── Src/                       # 源文件目录（与Inc/对应）
│       ├── main.c                 # 主程序入口和系统初始化
│       ├── freertos.c             # FreeRTOS任务创建和调度
│       ├── stm32f4xx_it.c         # 中断服务程序（ISR）
│       ├── stm32f4xx_hal_msp.c    # HAL MSP回调（DMA/NVIC配置）
│       └── [其他.c文件与头文件对应]
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/      # STM32 HAL库
│   └── CMSIS/                     # ARM CMSIS标准库
├── Middlewares/
│   └── Third_Party/FreeRTOS/      # FreeRTOS内核源码
├── MDK-ARM/
│   ├── v1.0.uvprojx               # Keil µVision项目文件（使用此文件编译）
│   └── RTE/                       # Run-Time Environment配置
└── v1.0.ioc                       # STM32CubeMX配置文件
```

## 构建和编译 (Build & Compilation)

### 使用Keil µVision编译
1. 使用Keil µVision打开 `MDK-ARM/v1.0.uvprojx`
2. Project -> Build Target
3. 或使用快捷键 F7

### 编程到芯片
1. 连接ST-Link调试器
2. 在Keil中: Flash -> Download
3. 按 Ctrl+F10

### 调试
1. Flash -> Download
2. Debug -> Start/Stop Debug Session
3. 或按 Ctrl+F5

## 代码组织规范 (Code Organization)

### STM32CubeMX与手动代码分离
项目使用STM32CubeMX生成的代码注释标记：
- `/* USER CODE BEGIN ... */` 和 `/* USER CODE END ... */` 包裹自定义代码
- **重要:** 只在这些标记区域内添加代码，避免在CubeMX重新生成时丢失修改
- GPIO初始化（gpio.c）是由CubeMX生成的，不应手动修改

### 推荐模块化结构
创建新的模块时，遵循以下结构：
```c
// led.h
#ifndef __LED_H__
#define __LED_H__
#include "main.h"

// 初始化函数
void LED_Init(void);

// 控制函数
void LED1_On(void);
void LED1_Off(void);
void LED1_Toggle(void);

#endif
```

## 常见开发任务 (Common Development Tasks)

### 添加新任务到FreeRTOS
编辑 `Core/Src/freertos.c` 中的 `MX_FREERTOS_Init()` 函数：
```c
/* USER CODE BEGIN RTOS_THREADS */
osThreadNew(MyTaskFunction, NULL, &attributes_MyTask);
/* USER CODE END RTOS_THREADS */
```

### 更改系统时钟或HAL配置
1. 打开 `v1.0.ioc` 使用STM32CubeMX (需要安装CubeMX)
2. 修改配置后，使用 "Generate Code" 以更新main.c和gpio.c
3. 检查USER CODE sections是否被保留

### GPIO操作快速参考
```c
// 设置高电平
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

// 设置低电平
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

// 读取引脚状态
GPIO_PinState state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);

// 翻转引脚
HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
```

## 关键依赖和配置

- **STM32F4 HAL库**: 包含在 Drivers/STM32F4xx_HAL_Driver
- **FreeRTOS**: 配置文件为 Core/Inc/FreeRTOSConfig.h
- **系统时钟**: 168MHz (参见SystemClock_Config()在main.c)

## 更新日志 (Update Log)

### 2026-05-17 (限位开关模块)
- ✅ `limit`：新增三路限位开关封装（PE15/PE8/PE7），提供原始电平、触发状态、批量读取与边沿检测接口

### 2026-04-07 (v1.0 初始版本)
- ✅ 创建CLAUDE.md文档
- ✅ 建立项目结构说明
- ✅ 记录GPIO硬件映射（LED和按键对应关系）
- ✅ 编写模块化LED控制函数
- ✅ 编写模块化按键输入函数
- 计划后续: 集成消抖算法、状态机实现更复杂的按键逻辑

### 2026-04-07 (维护性整理)
- ✅ `button`：消抖/等待按键的延时改为RTOS感知（RTOS运行时用`osDelay`，否则用`HAL_Delay`），并修复`Button_ReadAll()`错误返回的边界情况
- ✅ `oled`：补齐”绘制到buffer”和”刷新到屏幕”两层API（新增`OLED_DrawString/OLED_DrawNum/OLED_ClearBuffer/OLED_Flush`），保留原有`OLED_Show*`立即刷新行为
- ✅ `freertos`：将`oledTestTask`重命名为`oledTask`并补充注释，减少”测试命名”遗留

### 2026-04-07 (Bugfix)
- ✅ `i2c/it/oled`：补齐 I2C1 EV/ER 中断与 NVIC 使能，修复 `HAL_I2C_Master_Transmit_DMA()` 可能不完成导致 `OLED_Init()` 失败；同时将 `OLED_WaitI2CReady()` 改为 RTOS 运行时使用 `osDelay(1)`，避免忙等导致”CPU看起来只跑OLED任务”

### 2026-04-08 (鲁棒性和性能优化)
- ✅ **SC16IS752驱动关键修复**：
  - 修复DMA忙等待死锁问题：优化pending_mask处理逻辑，避免通道数据丢失
  - 改进DMA完成回调：先清busy标志再处理pending，消除竞态条件
  - 添加FIFO错误检测和自动恢复：检测溢出/帧错误并自动复位FIFO
  - 优化pending通道处理：按优先级顺序处理，避免某通道饿死
- ✅ **UART DMA接收增强**：
  - 添加自动重启机制：IDLE中断后自动重新启动DMA接收
  - 实现错误回调处理：捕获溢出/帧错误并自动恢复
  - 防止DMA停滞：确保接收循环持续运行
- ✅ **看门狗和超时保护**：
  - SC16任务添加5秒超时机制：避免永久阻塞
  - 超时计数器：连续超时15秒时记录，便于调试
  - 替换portMAX_DELAY为有限超时：提高系统鲁棒性
- ✅ **性能优化**：
  - 减少舵机初始化延时：从80ms优化到50ms
  - 优化中断处理流程：减少不必要的寄存器读取
  - 改进错误计数：统一使用g_sc16_dbg_spi_err_cnt

### 2026-04-10 (SC16IS752双通道并发读取死锁修复 - 方案2重构完成)
- ✅ **问题现象**：SC16IS752同时读取激光4（通道A，230400bps）和SBUS接收机（通道B，100000bps）时，正常运行约10秒后卡住，无法继续接收数据
- 🔍 **根因分析**：
  - **核心问题**：`pending_mask`机制存在设计缺陷，导致pending标志被反复设置但无法清除
  - **触发条件**：当DMA忙时EXTI中断触发，原设计将两个通道都标记为pending（0x03），但实际可能只有一个通道有中断
  - **死锁形成**：在DMA完成回调处理pending时，如果某通道RXLVL为0（数据已被读走或中断失效），该通道pending被清除，但另一通道pending仍保留；同时IRQ引脚可能因另一通道有数据而保持低电平，导致在兜底机制中重新调用`sc16_service_irq_from_isr`，又将刚处理完的通道重新标记为pending，形成死循环
  - **EXTI边沿触发限制**：EXTI配置为下降沿触发，如果在DMA忙时错过边沿，即使IRQ引脚保持低电平也不会再次触发中断
- ✅ **方案2重构（已实施）**：
  - **废除pending_mask机制**：完全移除软件pending状态变量，改用FreeRTOS事件标志组（EventGroup）
  - **EXTI中断简化**：中断回调仅设置事件标志唤醒任务，不做任何通道判断或状态设置
  - **任务层重构**：
    - 任务被事件标志唤醒后，主动读取两个通道的IIR寄存器判断中断源
    - 根据IIR返回值确定哪个通道有数据，然后调用`SC16_Poll()`触发DMA读取
    - IRQ引脚兜底机制：处理完IIR后检查IRQ引脚，如果仍为低电平则再次调用`SC16_Poll()`
  - **DMA完成回调优化**：DMA完成后检查IRQ引脚，如果为低则设置事件标志重新唤醒任务
  - **关键改进**：
    - 消除了pending_mask的竞态条件和死锁风险
    - 完全依赖芯片硬件IIR寄存器驱动状态机，避免软件状态不一致
    - 保留EXTI边沿触发，通过IRQ引脚轮询兜底解决边沿丢失问题
- 📝 **修改文件清单**：
  - `Core/Src/freertos.c`: 添加EventGroup创建和SC16_EVENT_IRQ_TRIGGERED定义
  - `Core/Inc/sc16_tasks.h`: 导出EventGroup句柄和事件标志定义
  - `Core/Src/stm32f4xx_it.c`: 简化EXTI回调注释
  - `Core/Src/sc16is752.c`: 
    - 移除`s_pending_mask`、`s_need_rescan`、`s_rr_next`静态变量
    - 重构`sc16_service_irq_from_isr()`：基于IIR驱动，优先通道A
    - 重构`SC16_EXTI_FallingCallbackFromISR()`：仅设置事件标志
    - 重构`SC16_SPI_TxRxCpltCallbackFromISR()`：DMA完成后检查IRQ引脚
    - 重构`SC16_SPI_ErrorCallbackFromISR()`：错误恢复后检查IRQ引脚
    - 简化`SC16_Poll()`：直接调用service函数
  - `Core/Src/sc16_tasks.c`: 
    - 任务主循环改为等待事件标志（5秒超时）
    - 被唤醒后读取IIR判断中断源
    - 调用`SC16_Poll()`处理数据
    - IRQ引脚兜底检查
- 💡 **设计优势**：
  - 架构清晰：EXTI唤醒 → 任务读IIR → 调用service → DMA读取 → 兜底检查
  - 无竞态条件：不再依赖软件pending状态，完全由硬件IIR驱动
  - 鲁棒性强：多层兜底机制（任务IIR轮询 + IRQ引脚检查 + DMA完成检查）
  - 易于调试：状态机简单，调试变量仍保留用于观察

## 调试和故障排查 (Debugging & Troubleshooting)

### SC16IS752调试观测点
在Keil Watch窗口添加以下变量，快速定位串口问题：

**基础计数器**：
- `g_sc16_dbg_exti_cnt`: IRQ#中断触发次数（为0说明硬件中断未触发）
- `g_sc16_dbg_irq_service_cnt`: 中断服务函数执行次数
- `g_sc16_dbg_dma_cplt_cnt`: SPI DMA完成次数（为0说明DMA未启动/未完成）
- `g_sc16_dbg_spi_err_cnt`: SPI/FIFO错误计数

**寄存器状态**：
- `g_sc16_dbg_last_iir[0]` / `[1]`: 最近一次读取的IIR寄存器（中断源识别）
- `g_sc16_dbg_last_lsr[0]` / `[1]`: 最近一次读取的LSR寄存器（线路状态）
- `g_sc16_dbg_last_rxlvl[0]` / `[1]`: 最近一次读取的RXLVL（FIFO字节数）

**SPI连通性测试**：
- `g_sc16_dbg_spr_ok[0]` / `[1]`: SPR读写测试结果（1=通过，0=失败）

### 常见问题诊断

**问题1：激光雷达数据不更新**
1. 检查 `g_LidarArray[0-3].is_updated` 是否为1
2. 对于UART通道（0-2）：检查DMA是否正常运行
3. 对于SC16通道（3）：检查上述SC16调试变量
4. 查看 `g_LidarArray[x].status`：0=正常，1=CRC错误，2=帧格式错误

**问题2：SC16IS752无数据**
1. `g_sc16_dbg_exti_cnt` 为0 → 检查IRQ#引脚连接和EXTI配置
2. `g_sc16_dbg_irq_service_cnt` 为0但exti_cnt>0 → 检查中断回调是否正确转发
3. `g_sc16_dbg_dma_cplt_cnt` 为0 → 检查SPI DMA配置和NVIC使能
4. `g_sc16_dbg_spr_ok` 为0 → SPI通信失败，检查CS/CLK/MOSI/MISO连接

**问题3：UART DMA停滞**
- 现已添加自动恢复机制：IDLE中断后自动重启DMA
- 错误回调会清除错误标志并重启接收
- 如仍有问题，检查DMA中断优先级配置

**问题4：任务超时告警**
- SC16任务现在使用5秒超时，连续3次超时（15秒）会记录
- 检查硬件连接和波特率配置
- 使用调试变量确认数据流是否正常

## 注意事项 (Important Notes)

1. **STM32CubeMX警告**: 重新生成代码会覆盖USER CODE区域外的所有内容
2. **FreeRTOS**: 使用CMSIS-OS v2 API (osThreadNew等)
3. **中断优先级**: 
   - EXTI (SC16 IRQ#): 优先级2（高优先级，快速响应）
   - SPI DMA: 优先级5（中等优先级）
   - UART DMA: 优先级6（较低优先级）
4. **时钟配置**: 系统运行在168MHz，某些外设需要分频设置
5. **串口鲁棒性**: 所有串口接收均已添加错误恢复和自动重启机制
6. **超时保护**: SC16任务使用5秒超时，避免永久阻塞
