/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    button.h
  * @brief   按键输入模块 - 提供按键操作的模块化接口（支持消抖）
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BUTTON_H__
#define __BUTTON_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ----------------------------- Types & Consts ----------------------------- */

/**
  * @brief 按键编号（与硬件丝印/接线一致：KEY1~KEY4）
  */
typedef enum
{
  BUTTON_KEY1 = 1,
  BUTTON_KEY2 = 2,
  BUTTON_KEY3 = 3,
  BUTTON_KEY4 = 4,
} Button_Key_t;

/**
  * @brief 按键状态
  * @note  本项目按键为低电平按下（GPIO读到RESET表示按下）
  */
typedef enum
{
  BUTTON_RELEASED = 0,
  BUTTON_PRESSED  = 1,
} Button_State_t;

#define BUTTON_KEY_COUNT 4u

/* 按键定义 */
#define KEY1_PORT    GPIOE
#define KEY1_PIN     GPIO_PIN_3

#define KEY2_PORT    GPIOE
#define KEY2_PIN     GPIO_PIN_4

#define KEY3_PORT    GPIOE
#define KEY3_PIN     GPIO_PIN_5

#define KEY4_PORT    GPIOE
#define KEY4_PIN     GPIO_PIN_6

/* 按键状态定义 */
#define KEY_RELEASED   ((int)BUTTON_RELEASED)
#define KEY_PRESSED    ((int)BUTTON_PRESSED)

/**
  * @brief  初始化按键模块
  * @retval None
  */
void Button_Init(void);

/**
  * @brief  扫描指定按键，返回当前状态（无消抖）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval KEY_PRESSED(1) 或 KEY_RELEASED(0)，错误时返回-1
  */
int Button_Read(uint8_t key);

/**
  * @brief  获取按键状态并进行消抖处理
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @param  debounce_ms: 消抖延迟时间（毫秒），建议20-50ms
  * @retval KEY_PRESSED(1) 或 KEY_RELEASED(0)，错误时返回-1
  * @note   此函数会产生延时：RTOS运行时内部使用osDelay，否则使用HAL_Delay
  */
int Button_ReadDebounce(uint8_t key, uint32_t debounce_ms);

/**
  * @brief  检测按键按下事件（仅返回一次）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval 1=检测到按下, 0=未按下，-1=错误
  * @note   需要在主循环或任务中定期调用此函数以检测边界
  */
int Button_IsPressedEdge(uint8_t key);

/**
  * @brief  检测按键释放事件（仅返回一次）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval 1=检测到释放, 0=未释放，-1=错误
  * @note   需要在主循环或任务中定期调用此函数以检测边界
  */
int Button_IsReleasedEdge(uint8_t key);

/**
  * @brief  获取所有按键状态
  * @param  states: 长度至少为4的数组，存储4个按键的状态
  * @retval 0=成功, -1=参数错误
  */
int Button_ReadAll(uint8_t *states);

/**
  * @brief  等待指定按键被按下（阻塞函数）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval 0=成功, -1=错误
  * @note   此函数会阻塞直到按键被按下；RTOS运行时内部使用osDelay，否则使用HAL_Delay
  */
int Button_WaitPress(uint8_t key);

#ifdef __cplusplus
}
#endif

#endif /* __BUTTON_H__ */
