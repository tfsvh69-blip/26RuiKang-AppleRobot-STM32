#include "sc16_tasks.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "sc16is752.h"
#include "lidar_manager.h"
#include "sbus.h"

/**
 * @brief SC16通道A接收任务（激光雷达4）
 * @note  方案2重构优化：事件标志唤醒后立即调用Poll，然后处理数据
 */
void StartSc16RxATask(void *argument)
{
  (void)argument;
  uint8_t rx_buf[256];
  uint32_t timeout_cnt = 0;

  for (;;)
  {
    /* 等待IRQ事件标志（100ms超时，快速响应） */
    EventBits_t bits = xEventGroupWaitBits(g_sc16EventGroup,
                                           SC16_EVENT_IRQ_TRIGGERED,
                                           pdFALSE,  /* 不清除标志，让两个任务都能看到 */
                                           pdFALSE,
                                           pdMS_TO_TICKS(100));

    if (bits & SC16_EVENT_IRQ_TRIGGERED)
    {
      /* 被IRQ唤醒，调用Poll处理（内部会读取IIR并启动DMA） */
      SC16_Poll(&g_sc16);

      /* 手动清除事件标志（在Poll之后） */
      xEventGroupClearBits(g_sc16EventGroup, SC16_EVENT_IRQ_TRIGGERED);

      timeout_cnt = 0;
    }
    else
    {
      /* 超时：100ms内没有IRQ事件，也尝试Poll一次（兜底） */
      SC16_Poll(&g_sc16);

      timeout_cnt++;
      if (timeout_cnt >= 50u)  /* 5秒 */
      {
        timeout_cnt = 0;
      }
    }

    /* 尝试从环形缓冲区读取数据 */
    if (xSemaphoreTake(g_sc16RxSemA, 0) == pdTRUE)
    {
      for (uint8_t loop = 0u; loop < 8u; loop++)
      {
        uint16_t want = (uint16_t)sizeof(rx_buf);
        (void)SC16_GetRxData(SC16_CH_A, rx_buf, &want);

        if (want == 0u)
          break;

        Lidar_ProcessStream(3u, rx_buf, want);
      }
    }

    osDelay(1);
  }
}

/**
 * @brief SC16通道B接收任务（SBUS接收机）
 * @note  方案2重构优化：仅负责数据消费，Poll由通道A任务统一处理
 */
void StartSc16RxBTask(void *argument)
{
  (void)argument;
  uint8_t rx_buf[64];

  for (;;)
  {
    /* 等待数据信号量（20ms超时） */
    if (xSemaphoreTake(g_sc16RxSemB, pdMS_TO_TICKS(20)) == pdTRUE)
    {
      for (uint8_t loop = 0u; loop < 8u; loop++)
      {
        uint16_t want = (uint16_t)sizeof(rx_buf);
        (void)SC16_GetRxData(SC16_CH_B, rx_buf, &want);

        if (want == 0u)
          break;

        SBUS_ProcessStream(rx_buf, want);
      }
    }

    osDelay(1);
  }
}
