#include "sc16is752.h"
#include "sc16_tasks.h"
#include <string.h>

/* ========================= 内部静态存储区 =========================
 * 说明：RingBuffer（环形缓冲区）的实际存储区在驱动内部进行静态分配。
 * 这样做可以避免在嵌入式系统中使用动态内存分配（malloc/free）带来的内存碎片和不确定性风险。
 */
static uint8_t s_rx_buf_a[SC16_RX_RING_SIZE_A];
static uint8_t s_rx_buf_b[SC16_RX_RING_SIZE_B];

/* 全局的 SC16IS752 句柄实例，用于保存设备的状态、SPI句柄、DMA缓冲区及信号量等信息 */
SC16_Handle_t g_sc16;

/* ========================= Keil Watch 调试变量 ========================= 
 * 以下变量主要用于在调试器（如Keil的Watch窗口）中实时监控系统的运行状态，
 * 方便排查中断丢失、DMA异常或SPI通信错误等问题。
 */
volatile uint32_t g_sc16_dbg_exti_cnt = 0;          /* 记录 EXTI（外部中断）触发的次数 */
volatile uint32_t g_sc16_dbg_irq_service_cnt = 0;   /* 记录 IRQ 处理函数被调用的次数 */
volatile uint32_t g_sc16_dbg_dma_cplt_cnt = 0;      /* 记录 DMA 传输完成中断的次数 */
volatile uint32_t g_sc16_dbg_spi_err_cnt = 0;       /* 记录 SPI 或 FIFO 发生错误的次数 */

volatile uint8_t g_sc16_dbg_last_iir[2] = {0, 0};   /* 记录通道A和B最后一次读取的 IIR（中断标识寄存器）值 */
volatile uint8_t g_sc16_dbg_last_lsr[2] = {0, 0};   /* 记录通道A和B最后一次读取的 LSR（线状态寄存器）值 */
volatile uint8_t g_sc16_dbg_last_rxlvl[2] = {0, 0}; /* 记录通道A和B最后一次读取的 RXLVL（接收FIFO级别）值 */
volatile uint8_t g_sc16_dbg_last_dma_ch = 0;        /* 记录上一次启动 DMA 传输的通道号 */
volatile uint8_t g_sc16_dbg_last_dma_len = 0;       /* 记录上一次启动 DMA 传输的数据长度 */

volatile uint8_t g_sc16_dbg_spr_ok[2] = {0, 0};     /* 记录通道A和B的 SPR 寄存器读写测试是否通过 */
volatile uint8_t g_sc16_dbg_spr_wr[2] = {0, 0};     /* 记录写入 SPR 寄存器的测试值 */
volatile uint8_t g_sc16_dbg_spr_rd[2] = {0, 0};     /* 记录从 SPR 寄存瓦读回的测试值 */

volatile uint32_t g_sc16_dbg_dma_start_cnt[2] = {0, 0};
volatile uint32_t g_sc16_dbg_dma_start_fail_cnt[2] = {0, 0};
volatile uint32_t g_sc16_dbg_dma_cplt_ch_cnt[2] = {0, 0};
volatile uint32_t g_sc16_dbg_dma_busy_skip_cnt = 0;
volatile uint32_t g_sc16_dbg_irq_retrigger_cnt = 0;
volatile uint32_t g_sc16_dbg_rto_rx0_cnt = 0;

/* 前向声明：避免 Keil(C90标准) 对后续的 static 函数产生“隐式声明”警告 */
static HAL_StatusTypeDef sc16_write_reg_nolock(SC16_Handle_t *h, uint8_t channel, uint8_t reg, uint8_t val);
static HAL_StatusTypeDef sc16_read_reg_nolock(SC16_Handle_t *h, uint8_t channel, uint8_t reg, uint8_t *val);

/**
 * @brief  测试指定通道的 SPR（Scratchpad Register，暂存寄存器）
 * @param  h: SC16 设备句柄
 * @param  channel: 通道号 (SC16_CH_A 或 SC16_CH_B)
 * @retval HAL_StatusTypeDef: 测试通过返回 HAL_OK，否则返回 HAL_ERROR
 * @note   SPR 是芯片内部最简单的“写后读回”寄存器，它不影响芯片的任何功能逻辑。
 * 通常用于在上电初始化时确认 MCU 与芯片之间的 SPI 物理连接（CS、CLK、MOSI、MISO）是否正常。
 * 如果 SPI 硬件接错，HAL库的 SPI 发送函数可能仍返回 HAL_OK，但读回的值必然与写入的值不匹配。
 */
static HAL_StatusTypeDef sc16_scratch_test_spr(SC16_Handle_t *h, uint8_t channel)
{
  /* 构造一个测试数据，通道A和通道B使用不同的异或掩码以区分 */
  uint8_t wr = (uint8_t)(0x5Au ^ (channel ? 0xA5u : 0x00u));
  uint8_t rd = 0x00u;

  /* 更新调试变量状态 */
  g_sc16_dbg_spr_wr[channel] = wr;
  g_sc16_dbg_spr_rd[channel] = 0x00u;
  g_sc16_dbg_spr_ok[channel] = 0u;

  /* 写入测试值到 SPR 寄存器 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_SPR, wr) != HAL_OK)
    return HAL_ERROR;
  
  /* 从 SPR 寄存器读取数值 */
  if (sc16_read_reg_nolock(h, channel, SC16_REG_SPR, &rd) != HAL_OK)
    return HAL_ERROR;

  g_sc16_dbg_spr_rd[channel] = rd;
  
  /* 校验写出值与读入值是否一致 */
  if (rd == wr)
  {
    g_sc16_dbg_spr_ok[channel] = 1u;
    return HAL_OK;
  }
  return HAL_ERROR;
}

/* 由于部分暴露给上层应用层的 API 内部没有传入句柄参数（例如 SC16_GetRxData），
 * 因此需要在驱动内部维护一个静态的”当前绑定”句柄指针。
 * 约束条件：当前工程硬件架构中只挂载了 1 片 SC16IS752 芯片。
 */
static SC16_Handle_t *s_sc16 = NULL;

/* 方案2重构：移除pending_mask机制，改用事件标志组唤醒 */

/* ========================= 内部工具函数 ========================= */

/**
 * @brief 拉低 CS (片选) 引脚，选中 SC16IS752 进行 SPI 通信
 */
static inline void sc16_cs_low(void)
{
  HAL_GPIO_WritePin(SC16_CS_GPIO_Port, SC16_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高 CS (片选) 引脚，结束 SC16IS752 的 SPI 通信
 */
static inline void sc16_cs_high(void)
{
  HAL_GPIO_WritePin(SC16_CS_GPIO_Port, SC16_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  初始化环形缓冲区 (Ring Buffer)
 * @param  rb: 指向环形缓冲区结构体的指针
 * @param  buf: 指向实际数据存储内存块的指针
 * @param  size: 缓冲区的总字节大小
 */
static void sc16_rb_init(SC16_RingBuffer_t *rb, uint8_t *buf, uint16_t size)
{
  rb->buf = buf;
  rb->size = size;
  rb->head = 0;       /* 写指针初始化 */
  rb->tail = 0;       /* 读指针初始化 */
  rb->drop_cnt = 0;   /* 丢包计数器清零 */
}

/**
 * @brief  在中断服务程序(ISR)中向环形缓冲区写入数据
 * @param  rb: 目标环形缓冲区
 * @param  data: 待写入的数据指针
 * @param  len: 待写入的数据长度
 * @note   使用了 FreeRTOS 的中断级临界区保护，防止在多中断嵌套时破坏缓冲区状态。
 */
static void sc16_rb_push_from_isr(SC16_RingBuffer_t *rb, const uint8_t *data, uint16_t len)
{
  /* 进入中断级临界区，保存当前中断屏蔽状态 */
  UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

  for (uint16_t i = 0; i < len; i++)
  {
    /* 计算下一个写指针的位置，如果到达末尾则折返到0 */
    uint16_t next = (uint16_t)(rb->head + 1u);
    if (next >= rb->size)
      next = 0u;

    /* 如果下一个写指针追上了读指针，说明缓冲区已满 */
    if (next == rb->tail)
    {
      /* 缓冲区满时的策略：丢弃新来的数据，不覆盖旧数据，并增加丢包计数 */
      rb->drop_cnt++;
      continue;
    }

    /* 写入数据并更新头指针 */
    rb->buf[rb->head] = data[i];
    rb->head = next;
  }

  /* 退出中断级临界区，恢复中断屏蔽状态 */
  taskEXIT_CRITICAL_FROM_ISR(saved);
}

/**
 * @brief  以线程安全的方式从环形缓冲区中提取(读取)数据
 * @param  rb: 目标环形缓冲区
 * @param  out: 数据提取后存放的目标缓冲指针
 * @param  max_len: 期望读取的最大字节数
 * @retval uint16_t: 实际成功读取的字节数
 * @note   在读取过程中使用了 FreeRTOS 任务级临界区，防止读取时被中断（如 DMA 完成中断）打断导致指针错乱。
 */
static uint16_t sc16_rb_pop_threadsafe(SC16_RingBuffer_t *rb, uint8_t *out, uint16_t max_len)
{
  uint16_t pulled = 0;

  /* 进入任务级临界区 */
  taskENTER_CRITICAL();
  while (pulled < max_len)
  {
    /* 如果读指针等于写指针，说明缓冲区已空，退出循环 */
    if (rb->tail == rb->head)
      break;

    /* 读取当前读指针指向的数据 */
    out[pulled++] = rb->buf[rb->tail];

    /* 更新读指针，到达末尾则折返 */
    uint16_t next = (uint16_t)(rb->tail + 1u);
    if (next >= rb->size)
      next = 0u;
    rb->tail = next;
  }
  /* 退出任务级临界区 */
  taskEXIT_CRITICAL();

  return pulled;
}

/**
 * @brief  执行一个短的、基础的 SPI 传输事务 (主要用于寄存器级读写)
 * @param  h: SC16 设备句柄
 * @param  tx0: 发送的第 1 个字节 (通常是命令/寄存器地址)
 * @param  tx1: 发送的第 2 个字节 (写入数据，或读取时作为提供时钟的 Dummy byte)
 * @param  rx1: 指向接收第 2 个字节的指针 (读取操作时使用)
 * @retval HAL_StatusTypeDef: SPI 传输状态
 * @note   这是一个两字节的短事务，由于耗时极短，允许在低优先级中断中被调用。
 */
static HAL_StatusTypeDef sc16_spi_xfer_byte(SC16_Handle_t *h, uint8_t tx0, uint8_t tx1, uint8_t *rx1)
{
  uint8_t tx[2] = {tx0, tx1};
  uint8_t rx[2] = {0, 0};

  /* 保护 SPI 外设访问：因为 EXTI(通常优先级较高, 如2) 可能会抢占 DMA ISR(优先级较低, 如5)，
   * 但由于此处代码是在不可抢占的上下文或临界区管理中调用，直接操作片选即可。
   */
  sc16_cs_low();
  /* 阻塞式传输2个字节，超时时间设置为 10ms */
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(h->hspi, tx, rx, 2, 10);
  sc16_cs_high();

  /* 如果传输成功且 rx1 指针不为空，则将读取到的数据（第二个周期的返回）赋值给目标 */
  if (st == HAL_OK && rx1)
    *rx1 = rx[1];
  return st;
}

/**
 * @brief  无锁的寄存器写操作
 */
static HAL_StatusTypeDef sc16_write_reg_nolock(SC16_Handle_t *h, uint8_t channel, uint8_t reg, uint8_t val)
{
  /* 构造 SPI 写命令字：组合寄存器地址和通道号 */
  uint8_t cmd = SC16_SPI_CMD_WRITE(reg, channel);
  return sc16_spi_xfer_byte(h, cmd, val, NULL);
}

/**
 * @brief  无锁的寄存器读操作
 */
static HAL_StatusTypeDef sc16_read_reg_nolock(SC16_Handle_t *h, uint8_t channel, uint8_t reg, uint8_t *val)
{
  /* 构造 SPI 读命令字：组合寄存器地址和通道号 */
  uint8_t cmd = SC16_SPI_CMD_READ(reg, channel);
  /* 读操作时，第二个字节发送 0xFF (Dummy data) 以产生时钟信号接收数据 */
  return sc16_spi_xfer_byte(h, cmd, 0xFFu, val);
}

/**
 * @brief  使能 SC16IS752 的增强型功能集 (Enhanced Register Set)
 * @param  h: SC16 设备句柄
 * @param  channel: 通道号
 * @retval HAL_StatusTypeDef
 * @note   根据数据手册要求，要访问增强寄存器集（如 EFR），必须先将 LCR 寄存器设置为 0xBF。
 * 而开启 EFR[4] 增强功能使能位后，才允许修改其他高级特性位（如 IER[7:4], FCR[5:4], MCR[7:5]）。
 */
static HAL_StatusTypeDef sc16_enable_enhanced(SC16_Handle_t *h, uint8_t channel)
{
  uint8_t old_lcr = 0;
  
  /* 读取并保存当前的 LCR (线控制寄存器) 值 */
  if (sc16_read_reg_nolock(h, channel, SC16_REG_LCR, &old_lcr) != HAL_OK)
    return HAL_ERROR;

  /* 写入魔法值 0xBF 到 LCR，进入增强寄存器集访问模式 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_LCR, SC16_LCR_ENHANCED) != HAL_OK)
    return HAL_ERROR;

  /* 读取当前的 EFR (增强功能寄存器) 值 */
  uint8_t efr = 0;
  if (sc16_read_reg_nolock(h, channel, SC16_REG_EFR, &efr) != HAL_OK)
    return HAL_ERROR;

  /* 置位 EFR[4] (Enhanced Functions Enable bit) */
  efr |= SC16_EFR_ENHANCED_EN;
  if (sc16_write_reg_nolock(h, channel, SC16_REG_EFR, efr) != HAL_OK)
    return HAL_ERROR;

  /* 恢复之前保存的 LCR 值，退出增强寄存器集访问模式 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_LCR, old_lcr) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/**
 * @brief  配置指定通道的 UART 波特率
 * @param  h: SC16 设备句柄
 * @param  channel: 通道号
 * @param  baud: 目标波特率 (bps)
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef sc16_set_baud(SC16_Handle_t *h, uint8_t channel, uint32_t baud)
{
  /*
   * 波特率分频器计算算法（依据芯片数据手册公式）：
   * uartclk (输入时钟频率)
   * divisor = -----------------------------------
   * prescaler(预分频,1或4) * 16 * baud
   *
   * 注意事项：
   * 1. 预分频器 (prescaler) 可以选 1 或 4，对应 MCR[7] (CLKSEL) 位。使能此位代表使用 /4 分频。
   * 2. MCR[7] 的写入操作必须在 EFR[4] 增强功能已使能的前提下才能成功。
   */
  if (baud == 0u || h->uartclk_hz == 0u)
    return HAL_ERROR;

  uint32_t prescaler = 1u;
  /* 默认按预分频系数为1进行计算 */
  uint32_t div = (h->uartclk_hz / 16u) / baud;

  /* 芯片的除数寄存器 DLL 和 DLH 合起来为 16 位，最大值为 65535。
   * 如果计算出的分频值超过了 16 位表达范围，则需要启用 /4 的预分频器以缩小除数值。
   */
  if (div >= 65536u)
  {
    prescaler = 4u;
    div /= prescaler;
  }

  /* 配置 MCR[7] 位以选择预分频器 */
  uint8_t mcr = 0;
  if (sc16_read_reg_nolock(h, channel, SC16_REG_MCR, &mcr) != HAL_OK)
    return HAL_ERROR;

  if (prescaler == 4u)
    mcr |= SC16_MCR_CLKSEL_DIV4;
  else
    mcr &= (uint8_t)~SC16_MCR_CLKSEL_DIV4;

  if (sc16_write_reg_nolock(h, channel, SC16_REG_MCR, mcr) != HAL_OK)
    return HAL_ERROR;

  /* 进入特殊寄存器集 (Special Register Set) 访问模式以配置波特率除数锁存器 DLL 和 DLH。
   * 条件：LCR[7] (DLAB) = 1 且 LCR 不能等于 0xBF。
   */
  uint8_t old_lcr = 0;
  if (sc16_read_reg_nolock(h, channel, SC16_REG_LCR, &old_lcr) != HAL_OK)
    return HAL_ERROR;

  /* 保留 LCR 原有低 7 位，将最高位 DLAB 置 1 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_LCR, (uint8_t)(SC16_LCR_DLAB | (old_lcr & 0x7Fu))) != HAL_OK)
    return HAL_ERROR;

  /* 分别写入除数的低八位 (DLL) 和高八位 (DLH) */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_DLL, (uint8_t)(div & 0xFFu)) != HAL_OK)
    return HAL_ERROR;
  if (sc16_write_reg_nolock(h, channel, SC16_REG_DLH, (uint8_t)((div >> 8) & 0xFFu)) != HAL_OK)
    return HAL_ERROR;

  /* 恢复原先的 LCR 值 (自动清除了 DLAB 位，恢复正常的读写模式) */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_LCR, old_lcr) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/**
 * @brief  配置指定通道的接收 FIFO 触发水位为 48 字节
 * @param  h: SC16 设备句柄
 * @param  channel: 通道号
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef sc16_config_fifo_trigger_48(SC16_Handle_t *h, uint8_t channel)
{
  /*
   * 需求目标：将 RX 触发水位设定为 48 字节（十进制），十六进制为 0x30。
   * 配置说明：SC16IS7xx 系列在使能 TCR/TLR 寄存器后，能够以更细的粒度配置触发级别。
   * 触发等级可以设置为 4 到 60 字节，步进为 4 字节。这里指定配置为 48 字节。
   */

  /* 使能 TCR/TLR 寄存器的访问权限：需置位 MCR[2] (前提是前面已经置位了 EFR[4]) */
  uint8_t mcr = 0;
  if (sc16_read_reg_nolock(h, channel, SC16_REG_MCR, &mcr) != HAL_OK)
    return HAL_ERROR;

  if (sc16_write_reg_nolock(h, channel, SC16_REG_MCR, (uint8_t)(mcr | SC16_MCR_TCRTLR_EN)) != HAL_OK)
    return HAL_ERROR;

  /* 获取当前 LCR 值，准备进入增强寄存器集访问 TLR */
  uint8_t old_lcr = 0;
  if (sc16_read_reg_nolock(h, channel, SC16_REG_LCR, &old_lcr) != HAL_OK)
    return HAL_ERROR;

  /* 写入 0xBF 到 LCR 切换到增强寄存器集 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_LCR, SC16_LCR_ENHANCED) != HAL_OK)
    return HAL_ERROR;

  /* 配置 TLR (Trigger Level Register): 
   * - 高 4 位用于配置 RX（接收）触发级。
   * - 低 4 位用于配置 TX（发送）触发级。
   * - 每 1 个单位代表 4 个字节。
   * 计算公式：RX 水平 48 字节 => 48 / 4 = 12 (即 0xC)。
   * 将 0xC 移位到高 4 位，结果为 0xC0。
   */
  uint8_t tlr = (uint8_t)(((48u / 4u) & 0x0Fu) << 4);
  if (sc16_write_reg_nolock(h, channel, SC16_REG_TLR, tlr) != HAL_OK)
    return HAL_ERROR;

  /* 恢复 LCR 寄存器值 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_LCR, old_lcr) != HAL_OK)
    return HAL_ERROR;

  /* 关闭 TCR/TLR 寄存器的访问权限 (清除 MCR[2])，防止后续操作被干扰 */
  if (sc16_write_reg_nolock(h, channel, SC16_REG_MCR, (uint8_t)(mcr & (uint8_t)~SC16_MCR_TCRTLR_EN)) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/**
 * @brief  在中断服务上下文中启动 RX（接收）的 DMA 传输
 * @param  h: SC16 设备句柄
 * @param  channel: 需要读取数据的通道
 * @param  rxlen: 需要读取的字节数（从 RXLVL 寄存器获取的 FIFO 深度）
 * @retval HAL_StatusTypeDef: 如果 DMA 空闲则启动并返回 HAL_OK，否则返回 HAL_BUSY
 */
static HAL_StatusTypeDef sc16_start_rx_dma_from_isr(SC16_Handle_t *h, uint8_t channel, uint8_t rxlen)
{
  if (rxlen == 0u)
    return HAL_OK;

  /* SC16IS752 的 FIFO 深度最大为 64 字节，防止参数越界访问 */
  if (rxlen > 64u)
    rxlen = 64u;

  /* 如果当前的 SPI DMA 通道正忙于传输前一次的数据 */
  if (h->dma_busy)
  {
    return HAL_BUSY;
  }

  g_sc16_dbg_dma_start_cnt[channel]++;

  /* 构造 DMA 发送缓冲数据：
   * 第 1 个字节：写入 RHR（接收保持寄存器）的读取指令。
   * 其余字节：填充 0xFF (Dummy bytes) 以产生 SPI 持续的时钟，从而将 FIFO 数据逐一读出。
   */
  h->tx_dma[channel][0] = SC16_SPI_CMD_READ(SC16_REG_RHR, channel);
  memset(&h->tx_dma[channel][1], 0xFF, rxlen);
  
  /* 清空接收缓冲区，准备接收（接收长度 = 指令字节 1 + 数据字节 rxlen） */
  memset(&h->rx_dma[channel][0], 0x00, (uint16_t)(rxlen + 1u));

  /* 标记状态为 DMA 忙，并记录通道号与长度，供 DMA 完成回调函数使用 */
  h->dma_busy = 1u;
  h->dma_ch = channel;
  h->dma_rxlen = rxlen;

  /* 拉低 CS 启动本次 SPI 多字节读取事务 */
  sc16_cs_low();
  
  /* 调用 HAL 库非阻塞式 SPI DMA 传输接口 */
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(h->hspi,
                                                     h->tx_dma[channel],
                                                     h->rx_dma[channel],
                                                     (uint16_t)(rxlen + 1u));
  if (st != HAL_OK)
  {
    /* 异常处理：如果 DMA 启动失败，必须马上拉高 CS 释放总线，
     * 清除繁忙标志，并增加错误计数以供调试。
     */
    sc16_cs_high();
    h->dma_busy = 0u;
    g_sc16_dbg_dma_start_fail_cnt[channel]++;
    g_sc16_dbg_spi_err_cnt++;
  }

  return st;
}

/**
 * @brief  在中断或回调上下文中服务 SC16 的硬件中断请求
 * @param  h: SC16 设备句柄
 * @note   方案2重构：完全基于IIR寄存器驱动，移除pending_mask机制
 */
static void sc16_service_irq_from_isr(SC16_Handle_t *h)
{
  /* 如果 SPI DMA 当前被占用，则不宜在此刻进行寄存器读取交互 */
  if (h->dma_busy)
  {
    g_sc16_dbg_dma_busy_skip_cnt++;
    return;
  }

  g_sc16_dbg_irq_service_cnt++;

  uint8_t rxlen_a = 0, rxlen_b = 0;
  uint8_t has_data_a = 0, has_data_b = 0;

  /* 第一步：读取两个通道的IIR寄存器，判断中断源 */
  for (uint8_t ch = 0; ch < 2u; ch++)
  {
    uint8_t iir = 0;
    if (sc16_read_reg_nolock(h, ch, SC16_REG_IIR, &iir) != HAL_OK)
      continue;

    g_sc16_dbg_last_iir[ch] = iir;

    /* IIR[0]=1 表示无中断 */
    if (iir & SC16_IIR_NO_INT_BIT)
      continue;

    /* 提取中断源ID */
    uint8_t int_id = (uint8_t)(iir & SC16_IIR_ID_MASK);

    /* 处理接收相关中断 */
    if (int_id == SC16_IIR_RDI_SRC ||
        int_id == SC16_IIR_RTOI_SRC ||
        int_id == SC16_IIR_RLSE_SRC ||
        int_id == SC16_IIR_XOFFI_SRC)
    {
      /* 读取LSR检查错误状态 */
      uint8_t lsr = 0;
      (void)sc16_read_reg_nolock(h, ch, SC16_REG_LSR, &lsr);
      g_sc16_dbg_last_lsr[ch] = lsr;

      /* 处理FIFO错误：溢出、奇偶校验错误、帧错误、Break */
      if (lsr & (SC16_LSR_OE | SC16_LSR_PE | SC16_LSR_FE | SC16_LSR_BI))
      {
        /* 重置接收FIFO以恢复 */
        (void)sc16_write_reg_nolock(h, ch, SC16_REG_FCR,
                                    (uint8_t)(SC16_FCR_FIFO_EN | SC16_FCR_RX_RESET));
        (void)sc16_write_reg_nolock(h, ch, SC16_REG_FCR, SC16_FCR_FIFO_EN);
        g_sc16_dbg_spi_err_cnt++;
        continue;
      }

      /* 读取RXLVL获取FIFO中的字节数 */
      uint8_t rxlen = 0;
      (void)sc16_read_reg_nolock(h, ch, SC16_REG_RXLVL, &rxlen);
      g_sc16_dbg_last_rxlvl[ch] = rxlen;

      /* 特殊处理：RTOI但RXLVL=0的情况，强制读1字节清除中断 */
      if ((int_id == SC16_IIR_RTOI_SRC) && (rxlen == 0u))
      {
        rxlen = 1u;
        g_sc16_dbg_rto_rx0_cnt++;
      }

      if (rxlen > 0u)
      {
        if (ch == SC16_CH_A)
        {
          rxlen_a = rxlen;
          has_data_a = 1u;
        }
        else
        {
          rxlen_b = rxlen;
          has_data_b = 1u;
        }
      }
    }
  }

  /* 第二步：如果有数据，启动DMA读取（优先通道A） */
  if (has_data_a)
  {
    HAL_StatusTypeDef st = sc16_start_rx_dma_from_isr(h, SC16_CH_A, rxlen_a);
    if (st == HAL_OK)
    {
      g_sc16_dbg_last_dma_ch = SC16_CH_A;
      g_sc16_dbg_last_dma_len = rxlen_a;
      return;  /* DMA启动成功，等待完成回调 */
    }
  }

  if (has_data_b)
  {
    HAL_StatusTypeDef st = sc16_start_rx_dma_from_isr(h, SC16_CH_B, rxlen_b);
    if (st == HAL_OK)
    {
      g_sc16_dbg_last_dma_ch = SC16_CH_B;
      g_sc16_dbg_last_dma_len = rxlen_b;
      return;  /* DMA启动成功，等待完成回调 */
    }
  }
}

/* ========================= 对外 API 实现 ========================= */

/**
 * @brief  向指定通道的寄存器写入一个字节数据
 * @note   提供给上层应用层的阻塞式寄存器写入接口。依赖初始化的全局 s_sc16 句柄。
 */
HAL_StatusTypeDef SC16_WriteReg(uint8_t channel, uint8_t reg, uint8_t val)
{
  if (!s_sc16)
    return HAL_ERROR;
  return sc16_write_reg_nolock(s_sc16, channel, reg, val);
}

/**
 * @brief  从指定通道的寄存器读取一个字节数据
 * @note   提供给上层应用层的阻塞式寄存器读取接口。
 */
HAL_StatusTypeDef SC16_ReadReg(uint8_t channel, uint8_t reg, uint8_t *val)
{
  if (!s_sc16)
    return HAL_ERROR;
  return sc16_read_reg_nolock(s_sc16, channel, reg, val);
}

/**
 * @brief  初始化 SC16IS752 芯片及其数据结构
 * @param  h: 待初始化的驱动句柄结构体
 * @param  hspi: STM32 HAL库底层的 SPI 控制器句柄
 * @param  uartclk_hz: 提供给 SC16 芯片的外接晶振频率或外部时钟频率 (Hz)
 * @param  rxSemA: 供通道A使用的接收完成同步信号量 (FreeRTOS)
 * @param  rxSemB: 供通道B使用的接收完成同步信号量 (FreeRTOS)
 * @retval HAL_StatusTypeDef: 初始化成功返回 HAL_OK
 */
HAL_StatusTypeDef SC16_Init(SC16_Handle_t *h,
                            SPI_HandleTypeDef *hspi,
                            uint32_t uartclk_hz,
                            SemaphoreHandle_t rxSemA,
                            SemaphoreHandle_t rxSemB)
{
  if (!h || !hspi)
    return HAL_ERROR;

  /* 初始化结构体及参数绑定 */
  memset(h, 0, sizeof(*h));
  h->hspi = hspi;
  h->uartclk_hz = uartclk_hz;
  h->rx_sem[SC16_CH_A] = rxSemA;
  h->rx_sem[SC16_CH_B] = rxSemB;

  /* 初始化通道 A 和通道 B 的应用层软件环形缓冲区 */
  sc16_rb_init(&h->rx_rb[SC16_CH_A], s_rx_buf_a, (uint16_t)sizeof(s_rx_buf_a));
  sc16_rb_init(&h->rx_rb[SC16_CH_B], s_rx_buf_b, (uint16_t)sizeof(s_rx_buf_b));

  /* 初始时强制拉高 SPI 片选引脚，处于空闲非选中状态 */
  sc16_cs_high();

  /* 将当前实例绑定到全局指针，允许 SC16_WriteReg 等无状态接口进行访问 */
  s_sc16 = h;

  /* 外部环境依赖：为了确保底层可以进行 SPI 的双向 DMA（即发送和接收 DMA 同时工作），
   * 必须在外部（通常在 stm32xxxx_hal_msp.c 中）开启对应的 DMA 中断响应。
   */
  extern void DMA2_Stream3_NVIC_Init(void);
  DMA2_Stream3_NVIC_Init();

  /* 通过写入 IOCONTROL[3]=1 对芯片执行软件复位。此操作会重置芯片内部所有的寄存器和状态机。 */
  (void)sc16_write_reg_nolock(h, SC16_CH_A, SC16_REG_IOCONTROL, SC16_IOCONTROL_SRESET);
  
  /* 硬件在复位后内部需要时间稳定，根据经验建议延时至少 1 毫秒，此处设置 2 毫秒余量 */
  HAL_Delay(2);

  /* SPI 物理连通性自检。分别对两个通道执行暂存寄存器读写测试。
   * 如果该步骤失败，无需继续后续配置，通常问题出在：
   * 1. 硬件引脚(CS/CLK/MOSI/MISO)连线断开或短路。
   * 2. 芯片未供电或时钟未起振。
   * 3. SPI 极性/相位 (CPOL/CPHA) 配置错误。
   */
  if (sc16_scratch_test_spr(h, SC16_CH_A) != HAL_OK)
    return HAL_ERROR;
  if (sc16_scratch_test_spr(h, SC16_CH_B) != HAL_OK)
    return HAL_ERROR;

  /* ---------- 具体通道配置环节 ---------- 
   * 业务需求：
   * 通道 A = 230400 波特率，8数据位，无校验，1停止位 (8N1)
   * 通道 B = 100000 波特率，8数据位，偶校验，2停止位 (8E2)
   */
  for (uint8_t ch = 0; ch < 2u; ch++)
  {
    /* 配置的第一步：先关闭该通道的所有中断，防止在寄存器配置的中间态触发了乱码中断响应 */
    (void)sc16_write_reg_nolock(h, ch, SC16_REG_IER, 0x00);

    /* 使能增强寄存器功能，为后面配置分频器及触发水平铺垫 */
    if (sc16_enable_enhanced(h, ch) != HAL_OK)
      return HAL_ERROR;

    /* 将接收 FIFO 的触发深度设置为 48 字节，优化 DMA 的使用效率并减少中断频率 */
    if (sc16_config_fifo_trigger_48(h, ch) != HAL_OK)
      return HAL_ERROR;

    /* 重置并使能该通道内部的发送(TX)和接收(RX) FIFO 硬件结构 */
    (void)sc16_write_reg_nolock(h, ch, SC16_REG_FCR, (uint8_t)(SC16_FCR_RX_RESET | SC16_FCR_TX_RESET));
    (void)sc16_write_reg_nolock(h, ch, SC16_REG_FCR, (uint8_t)(SC16_FCR_FIFO_EN));

    /* 独立配置各通道的波特率与数据帧格式 */
    if (ch == SC16_CH_A)
    {
      if (sc16_set_baud(h, ch, 230400u) != HAL_OK)
        return HAL_ERROR;

      /* LCR配置：8 数据位(WORD_LEN_8)。其他位默认0，对应无校验，1停止位 */
      (void)sc16_write_reg_nolock(h, ch, SC16_REG_LCR, SC16_LCR_WORD_LEN_8);
    }
    else
    {
      /* 重点工程提示：
       * 在外部晶振 uartclk = 14.7456MHz，且内部以 16 倍采样率的情况下，
       * 100000 bps 并不是一个能被完美整除的理想波特率。
       * (14745600 / 16) = 921600。
       * 921600 / 100000 = 9.216。
       * 驱动配置时如果写入除数 9，实际波特率将是 921600 / 9 = 102400 bps。
       * 这个计算结果与目标 100000 bps 存在约 2.4% 的偏差。如果对端的 SBUS/接收设备对波特率极其敏感，
       * 则可能需要考虑更换外部晶振频率或者采用主控MCU的其他时钟分频补偿方案。
       */
      if (sc16_set_baud(h, ch, 100000u) != HAL_OK)
        return HAL_ERROR;

      /* LCR配置：8 数据位，使能奇偶校验(PARITY_EN)，选择偶校验(EVEN_PARITY)，选择2个停止位(STOP_2) */
      (void)sc16_write_reg_nolock(h, ch, SC16_REG_LCR,
                                 (uint8_t)(SC16_LCR_WORD_LEN_8 | SC16_LCR_PARITY_EN | SC16_LCR_EVEN_PARITY | SC16_LCR_STOP_2));
    }

    /* 重新打开中断：
     * 使能 RX 可用中断 (RDI) 以便接收数据。
     * 使能接收线状态中断 (RLSI) 用于在底层监测帧错误或奇偶校验错误。
     */
    (void)sc16_write_reg_nolock(h, ch, SC16_REG_IER, (uint8_t)(SC16_IER_RDI | SC16_IER_RLSI));
  }

  /* 配置完成前进行一次状态清理：
   * 芯片在初始化时由于重置等操作，可能会产生虚假的寄存器挂起状态。
   * 读取一次 IIR 和 LSR 可借用读行为本身的副作用，彻底清除这些旧状态。
   */
  uint8_t dummy = 0;
  (void)sc16_read_reg_nolock(h, SC16_CH_A, SC16_REG_IIR, &dummy);
  (void)sc16_read_reg_nolock(h, SC16_CH_B, SC16_REG_IIR, &dummy);
  (void)sc16_read_reg_nolock(h, SC16_CH_A, SC16_REG_LSR, &dummy);
  (void)sc16_read_reg_nolock(h, SC16_CH_B, SC16_REG_LSR, &dummy);

  return HAL_OK;
}

/**
 * @brief  从指定通道的驱动级环形缓冲区中获取（消费）接收到的数据
 * @param  channel: 要提取数据的通道号
 * @param  buf: 应用层提供的，用于存放获取数据的内存指针
 * @param  len: [输入/输出] 参数。调用时表示最多想要提取的字节数；返回时表示实际提取到的字节数。
 * @retval HAL_StatusTypeDef: 参数检查通过返回 HAL_OK。注意返回 HAL_OK 不代表读到了数据，需判定 *len 是否大于 0。
 */
HAL_StatusTypeDef SC16_GetRxData(uint8_t channel, uint8_t *buf, uint16_t *len)
{
  if (!buf || !len)
    return HAL_ERROR;
  /* 仅支持双通道 (0 和 1) */
  if (channel > 1u)
    return HAL_ERROR;

  if (!s_sc16)
    return HAL_ERROR;

  uint16_t max_len = *len;
  /* 调用线程安全的出队函数，从对应的 Ring Buffer 中抽取数据 */
  uint16_t got = sc16_rb_pop_threadsafe(&s_sc16->rx_rb[channel], buf, max_len);
  
  /* 更新实际出队的数据量给调用方 */
  *len = got;

  return HAL_OK;
}

/**
 * @brief  当连接到 SC16 IRQ引脚的 MCU 外部中断 (EXTI) 发生下降沿时调用的回调函数
 * @param  h: SC16 设备句柄
 * @param  gpio_pin: 触发当前中断的引脚号
 * @note   方案2重构：仅设置事件标志唤醒任务，不做任何通道判断
 */
void SC16_EXTI_FallingCallbackFromISR(SC16_Handle_t *h, uint16_t gpio_pin)
{
  if (!h)
    return;

  /* 过滤出对应 SC16 芯片的中断引脚 */
  if (gpio_pin != SC16_IRQ_Pin)
    return;

  g_sc16_dbg_exti_cnt++;

  /* 方案2设计：仅设置事件标志，唤醒处理任务 */
  if (g_sc16EventGroup != NULL)
  {
    BaseType_t hpw = pdFALSE;
    (void)xEventGroupSetBitsFromISR(g_sc16EventGroup, SC16_EVENT_IRQ_TRIGGERED, &hpw);
    portYIELD_FROM_ISR(hpw);
  }
}

/**
 * @brief  当 SPI DMA 传输（发送+接收）全部完成时调用的回调函数
 * @param  h: SC16 设备句柄
 * @param  hspi: 完成此次传输的底层 SPI 外设句柄
 * @note   方案2重构：DMA完成后检查IRQ引脚，如果仍为低则设置事件标志
 */
void SC16_SPI_TxRxCpltCallbackFromISR(SC16_Handle_t *h, SPI_HandleTypeDef *hspi)
{
  if (!h || !hspi)
    return;

  /* 过滤无关 SPI 事件 */
  if (hspi != h->hspi)
    return;

  /* 重点：硬件层面结束。DMA 传输完毕后，必须马上拉高片选 (CS)，结束这一轮完整的 SPI 会话 */
  sc16_cs_high();

  g_sc16_dbg_dma_cplt_cnt++;

  uint8_t ch = h->dma_ch;
  uint8_t rxlen = h->dma_rxlen;

  if (ch < 2) {
    g_sc16_dbg_dma_cplt_ch_cnt[ch]++;
  }

  /* 先处理数据，再清除busy标志 */
  if (rxlen)
  {
    /* 由于在启动 DMA 时，我们发送的第一个字节是读取命令(Dummy=0)，
     * 对应的 DMA 接收缓冲区(rx_dma)第 0 个字节返回的是无效状态数据。
     * 因此，真正有效载荷的数据是从索引为 [1] 的位置开始提取的，填入该通道的环形缓冲区。
     */
    sc16_rb_push_from_isr(&h->rx_rb[ch], &h->rx_dma[ch][1], rxlen);

    /* 数据就绪：如果是基于 RTOS 系统运行，则释放信号量通知挂起在接收任务上的应用层线程进行消费 */
    if (h->rx_sem[ch] != NULL)
    {
      BaseType_t hpw = pdFALSE;
      /* GiveFromISR: RTOS中断级安全发送信号量 */
      (void)xSemaphoreGiveFromISR(h->rx_sem[ch], &hpw);
      /* 要求上下文切换(Yield) 以立刻让唤醒的高优先级任务抢占当前环境运行 */
      portYIELD_FROM_ISR(hpw);
    }
  }

  /* 在临界区内清除busy标志 */
  UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
  h->dma_busy = 0u;
  taskEXIT_CRITICAL_FROM_ISR(saved);

  /* 方案2兜底机制：DMA完成后检查IRQ引脚，如果仍为低则设置事件标志 */
  if (HAL_GPIO_ReadPin(SC16_IRQ_GPIO_Port, SC16_IRQ_Pin) == GPIO_PIN_RESET)
  {
    g_sc16_dbg_irq_retrigger_cnt++;
    if (g_sc16EventGroup != NULL)
    {
      BaseType_t hpw = pdFALSE;
      (void)xEventGroupSetBitsFromISR(g_sc16EventGroup, SC16_EVENT_IRQ_TRIGGERED, &hpw);
      portYIELD_FROM_ISR(hpw);
    }
  }
}

/**
 * @brief SPI 错误回调：避免 DMA 卡死
 * @note  方案2重构：错误恢复后检查IRQ引脚并设置事件标志
 */
void SC16_SPI_ErrorCallbackFromISR(SC16_Handle_t *h, SPI_HandleTypeDef *hspi)
{
  if (!h || !hspi)
    return;

  if (hspi != h->hspi)
    return;

  /* 增加错误排查计数器 */
  g_sc16_dbg_spi_err_cnt++;

  /* 容错与恢复机制：强行拉高CS片选，清空dma_busy标志 */
  sc16_cs_high();
  h->dma_busy = 0u;

  /* 方案2兜底：如果异常恢复后IRQ引脚仍为低，设置事件标志 */
  if (HAL_GPIO_ReadPin(SC16_IRQ_GPIO_Port, SC16_IRQ_Pin) == GPIO_PIN_RESET)
  {
    g_sc16_dbg_irq_retrigger_cnt++;
    if (g_sc16EventGroup != NULL)
    {
      BaseType_t hpw = pdFALSE;
      (void)xEventGroupSetBitsFromISR(g_sc16EventGroup, SC16_EVENT_IRQ_TRIGGERED, &hpw);
      portYIELD_FROM_ISR(hpw);
    }
  }
}

/**
 * @brief  轮询函数：供任务层调用，检查并处理SC16中断
 * @param  h: SC16 设备句柄
 * @note   方案2重构：简化为直接调用service函数
 */
void SC16_Poll(SC16_Handle_t *h)
{
  if (!h)
    return;

  /* 仅在DMA空闲时触发处理 */
  if (h->dma_busy)
    return;

  /* 直接调用中断服务函数处理 */
  sc16_service_irq_from_isr(h);
}
