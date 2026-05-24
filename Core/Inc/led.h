/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    led.h
  * @brief   LED控制模块 - 提供LED操作的模块化接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ----------------------------- Types & Consts ----------------------------- */

/**
  * @brief LED编号（与硬件丝印/接线一致：LED1~LED3）
  * @note  本板LED为高电平点亮
  */
typedef enum
{
  LED_ID_1 = 1,
  LED_ID_2 = 2,
  LED_ID_3 = 3,
} LED_Id_t;

#define LED_COUNT 3u

/* LED定义 */
#define LED1_PORT    GPIOE
#define LED1_PIN     GPIO_PIN_2

#define LED2_PORT    GPIOD
#define LED2_PIN     GPIO_PIN_13

#define LED3_PORT    GPIOD
#define LED3_PIN     GPIO_PIN_14

/**
  * @brief  初始化LED模块（由MX_GPIO_Init调用，此函数可选）
  * @retval None
  */
void LED_Init(void);

/**
  * @brief  点亮指定LED
  * @param  led: LED编号 (1, 2, 3)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_On(uint8_t led);

/**
  * @brief  熄灭指定LED
  * @param  led: LED编号 (1, 2, 3)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_Off(uint8_t led);

/**
  * @brief  翻转指定LED状态
  * @param  led: LED编号 (1, 2, 3)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_Toggle(uint8_t led);

/**
  * @brief  设置LED为特定状态
  * @param  led: LED编号 (1, 2, 3)
  * @param  state: GPIO_PIN_SET(点亮) 或 GPIO_PIN_RESET(熄灭)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_SetState(uint8_t led, GPIO_PinState state);

/**
  * @brief  获取LED当前状态
  * @param  led: LED编号 (1, 2, 3)
  * @retval GPIO_PIN_SET(点亮) 或 GPIO_PIN_RESET(熄灭)，错误时返回0xFF
  */
GPIO_PinState LED_GetState(uint8_t led);

/**
  * @brief  全部LED点亮
  * @retval None
  */
void LED_AllOn(void);

/**
  * @brief  全部LED熄灭
  * @retval None
  */
void LED_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */
