#ifndef __SC16IS752_H__
#define __SC16IS752_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * SC16IS752 (Dual UART bridge) 底层驱动
 * - 接口：SPI1, CPOL=0, CPHA=0, 8bit
 * - CS：PB12 (低有效)
 * - IRQ#：PB14 (下降沿 EXTI)
 *
 * 设计目标：极低 CPU 占用
 * 架构：EXTI 硬件中断 -> (短SPI读 IIR/LSR/RXLVL) -> SPI DMA 批量读 FIFO -> 回调压入 RingBuffer -> 释放信号量唤醒任务
 */

#include "FreeRTOS.h"
#include "semphr.h"

/* ========================= 基础宏定义 ========================= */

#define SC16_CH_A   (0u)
#define SC16_CH_B   (1u)

/* 默认 IRQ 引脚（如你在 CubeMX 里改了，请在工程全局宏覆盖） */
#ifndef SC16_IRQ_GPIO_Port
#define SC16_IRQ_GPIO_Port GPIOB
#endif
#ifndef SC16_IRQ_Pin
#define SC16_IRQ_Pin GPIO_PIN_14
#endif

/* SPI 指令格式（参考公开资料：SC16IS7xx SPI 指令）
 * bit7 : 1=读, 0=写
 * bit6..3 : 寄存器地址 A[3:0]
 * bit2..1 : 通道选择 CH[1:0]（本器件仅用到 bit1：0=A, 1=B）
 * bit0 : 0
 */
#define SC16_SPI_CMD_WRITE(reg, ch)   (uint8_t)((((reg) & 0x0Fu) << 3) | (((ch) & 0x01u) << 1))
#define SC16_SPI_CMD_READ(reg, ch)    (uint8_t)(0x80u | SC16_SPI_CMD_WRITE((reg), (ch)))

/* ========================= 寄存器定义（General Register Set） ========================= */
#define SC16_REG_RHR   (0x00u) /* 读：RX FIFO */
#define SC16_REG_THR   (0x00u) /* 写：TX FIFO */
#define SC16_REG_IER   (0x01u)
#define SC16_REG_IIR   (0x02u) /* 读 */
#define SC16_REG_FCR   (0x02u) /* 写 */
#define SC16_REG_LCR   (0x03u)
#define SC16_REG_MCR   (0x04u)
#define SC16_REG_LSR   (0x05u)
#define SC16_REG_MSR   (0x06u)
#define SC16_REG_SPR   (0x07u)
#define SC16_REG_TXLVL (0x08u)
#define SC16_REG_RXLVL (0x09u)
#define SC16_REG_IODIR (0x0Au)
#define SC16_REG_IOSTATE (0x0Bu)
#define SC16_REG_IOINTENA (0x0Cu)
#define SC16_REG_IOCONTROL (0x0Eu)
#define SC16_REG_EFCR  (0x0Fu)

/* ========================= Special Register Set（LCR[7]=1 且 LCR!=0xBF） ========================= */
#define SC16_REG_DLL   (0x00u)
#define SC16_REG_DLH   (0x01u)

/* ========================= Enhanced Register Set（LCR=0xBF） ========================= */
#define SC16_REG_EFR   (0x02u)
#define SC16_REG_TCR   (0x06u)
#define SC16_REG_TLR   (0x07u)

/* ========================= 位定义 ========================= */
/* IER */
#define SC16_IER_RDI   (1u << 0) /* 接收数据可用中断 */
#define SC16_IER_RLSI  (1u << 2) /* 接收线状态中断（错误/Break） */

/* IIR */
#define SC16_IIR_NO_INT_BIT (0x01u)
#define SC16_IIR_ID_MASK    (0x3Eu) /* bits[5:1] */
#define SC16_IIR_THRI_SRC   (0x02u)
#define SC16_IIR_RDI_SRC    (0x04u)
#define SC16_IIR_RLSE_SRC   (0x06u)
#define SC16_IIR_RTOI_SRC   (0x0Cu)
#define SC16_IIR_XOFFI_SRC  (0x10u)

/* FCR */
#define SC16_FCR_FIFO_EN     (1u << 0)
#define SC16_FCR_RX_RESET    (1u << 1)
#define SC16_FCR_TX_RESET    (1u << 2)

/* LCR */
#define SC16_LCR_WORD_LEN_8  (0x03u)
#define SC16_LCR_STOP_2      (1u << 2)
#define SC16_LCR_PARITY_EN   (1u << 3)
#define SC16_LCR_EVEN_PARITY (1u << 4)
#define SC16_LCR_DLAB        (1u << 7)
#define SC16_LCR_ENHANCED    (0xBFu)

/* MCR */
#define SC16_MCR_TCRTLR_EN   (1u << 2)
#define SC16_MCR_CLKSEL_DIV4 (1u << 7) /* 时钟/4 预分频（写使能依赖 EFR[4]） */

/* LSR */
#define SC16_LSR_DR          (1u << 0)
#define SC16_LSR_OE          (1u << 1)
#define SC16_LSR_PE          (1u << 2)
#define SC16_LSR_FE          (1u << 3)
#define SC16_LSR_BI          (1u << 4)
#define SC16_LSR_FIFOE       (1u << 7)

/* IOCONTROL */
#define SC16_IOCONTROL_SRESET (1u << 3)

/* EFR */
#define SC16_EFR_ENHANCED_EN  (1u << 4)

/* ========================= RingBuffer ========================= */

typedef struct
{
  uint8_t *buf;
  uint16_t size;
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint32_t drop_cnt;
} SC16_RingBuffer_t;

/* ========================= 配置/句柄 ========================= */

#ifndef SC16_RX_RING_SIZE_A
#define SC16_RX_RING_SIZE_A  (2048u)  /* 激光高频流：建议更大 */
#endif

#ifndef SC16_RX_RING_SIZE_B
#define SC16_RX_RING_SIZE_B  (512u)   /* SBUS：25B/帧 */
#endif

typedef struct
{
  SPI_HandleTypeDef *hspi;

  SemaphoreHandle_t rx_sem[2];

  SC16_RingBuffer_t rx_rb[2];

  /* SPI DMA 临时缓冲（tx/rx均需要产生时钟）
   * 注意：第0字节为 SPI 指令字节，真实FIFO数据从 [1] 开始
   */
  uint8_t tx_dma[2][65];
  uint8_t rx_dma[2][65];

  volatile uint8_t dma_busy;
  volatile uint8_t dma_ch;
  volatile uint8_t dma_rxlen;

  uint32_t uartclk_hz; /* XTAL 输入频率，例如 14745600Hz */
} SC16_Handle_t;

/* 默认句柄：工程只挂 1 颗 SC16IS752 时可直接使用 */
extern SC16_Handle_t g_sc16;

/* ========================= 调试观测点（Keil Watch 友好） =========================
 * 用途：当 SBUS 没解析出来时，快速分层定位“IRQ有没有来 / DMA有没有跑 / FIFO里有没有字节”。
 * 建议 Watch：
 * - g_sc16_dbg_exti_cnt：PB14 IRQ# 下降沿次数（为0说明中断没触发）
 * - g_sc16_dbg_dma_cplt_cnt：SPI DMA 完成次数（为0说明没启动/没完成）
 * - g_sc16_dbg_last_iir/lsr/rxlvl：最近一次服务IRQ时读到的寄存器值
 */
extern volatile uint32_t g_sc16_dbg_exti_cnt;
extern volatile uint32_t g_sc16_dbg_irq_service_cnt;
extern volatile uint32_t g_sc16_dbg_dma_cplt_cnt;
extern volatile uint32_t g_sc16_dbg_spi_err_cnt;

extern volatile uint8_t g_sc16_dbg_last_iir[2];
extern volatile uint8_t g_sc16_dbg_last_lsr[2];
extern volatile uint8_t g_sc16_dbg_last_rxlvl[2];
extern volatile uint8_t g_sc16_dbg_last_dma_ch;
extern volatile uint8_t g_sc16_dbg_last_dma_len;

/* SPI连通性自检：写/读回 SPR（Scratchpad Register）
 * g_sc16_dbg_spr_ok[ch]=1 表示该通道 SPR 读回匹配。
 */
extern volatile uint8_t g_sc16_dbg_spr_ok[2];
extern volatile uint8_t g_sc16_dbg_spr_wr[2];
extern volatile uint8_t g_sc16_dbg_spr_rd[2];

/* 新增：深度状态调试指标 */
extern volatile uint32_t g_sc16_dbg_dma_start_cnt[2];       /* 每个通道触发DMA读取的次数 */
extern volatile uint32_t g_sc16_dbg_dma_start_fail_cnt[2];  /* 触发DMA读取失败的次数 */
extern volatile uint32_t g_sc16_dbg_dma_cplt_ch_cnt[2];     /* 每个通道DMA完成的次数 */
extern volatile uint32_t g_sc16_dbg_dma_busy_skip_cnt;      /* 因繁忙导致跳过处理的次数 */
extern volatile uint32_t g_sc16_dbg_irq_retrigger_cnt;      /* completion里因引脚仍为低而重新触发的次数 */
extern volatile uint32_t g_sc16_dbg_rto_rx0_cnt;            /* 读取到超时中断但RXLVL为0的伪中断防御次数 */

/* ========================= API 声明 ========================= */

/**
 * @brief  初始化 SC16IS752
 * @param  h           驱动句柄（由调用者提供常驻内存）
 * @param  hspi        SPI句柄（一般为 &hspi1）
 * @param  uartclk_hz  SC16 晶振频率（本项目为 14745600）
 * @param  rxSemA      通道A 接收信号量（外部创建后传入）
 * @param  rxSemB      通道B 接收信号量（外部创建后传入）
 * @retval HAL status
 */
HAL_StatusTypeDef SC16_Init(SC16_Handle_t *h,
                            SPI_HandleTypeDef *hspi,
                            uint32_t uartclk_hz,
                            SemaphoreHandle_t rxSemA,
                            SemaphoreHandle_t rxSemB);

/**
 * @brief  线程安全读取接收数据（从RingBuffer取数据）
 * @param  channel 0=A,1=B
 * @param  buf     目标缓冲
 * @param  len     入参：buf最大长度；出参：实际读到的长度
 */
HAL_StatusTypeDef SC16_GetRxData(uint8_t channel, uint8_t *buf, uint16_t *len);

/**
 * @brief  寄存器写抽象（自动拉低/拉高CS）
 */
HAL_StatusTypeDef SC16_WriteReg(uint8_t channel, uint8_t reg, uint8_t val);

/**
 * @brief  寄存器读抽象（自动拉低/拉高CS）
 */
HAL_StatusTypeDef SC16_ReadReg(uint8_t channel, uint8_t reg, uint8_t *val);

/* ========================= 回调转发（在 HAL 回调里调用） ========================= */

/**
 * @brief  EXTI 下降沿中断到来时调用（建议在 HAL_GPIO_EXTI_Callback 中转发）
 */
void SC16_EXTI_FallingCallbackFromISR(SC16_Handle_t *h, uint16_t gpio_pin);

/**
 * @brief  SPI TxRx DMA 完成回调（建议在 HAL_SPI_TxRxCpltCallback 中转发）
 */
void SC16_SPI_TxRxCpltCallbackFromISR(SC16_Handle_t *h, SPI_HandleTypeDef *hspi);

/**
 * @brief  SPI 错误回调（可选）
 */
void SC16_SPI_ErrorCallbackFromISR(SC16_Handle_t *h, SPI_HandleTypeDef *hspi);

/**
 * @brief  主动轮询SC16中断线状态（可在任务上下文周期调用，用于边沿丢失时恢复）
 */
void SC16_Poll(SC16_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* __SC16IS752_H__ */
