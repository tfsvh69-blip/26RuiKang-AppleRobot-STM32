/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    led.c
  * @brief   LED控制模块实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  初始化LED模块
  * @retval None
  */
void LED_Init(void)
{
  /* LED初始化已由MX_GPIO_Init完成，此函数保留用于可能的扩展 */
}

/**
  * @brief  点亮指定LED
  * @param  led: LED编号 (1, 2, 3)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_On(uint8_t led)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
      return 0;
    case 2:
      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
      return 0;
    case 3:
      HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET);
      return 0;
    default:
      return -1;  /* 参数错误 */
  }
}

/**
  * @brief  熄灭指定LED
  * @param  led: LED编号 (1, 2, 3)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_Off(uint8_t led)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
      return 0;
    case 2:
      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
      return 0;
    case 3:
      HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET);
      return 0;
    default:
      return -1;  /* 参数错误 */
  }
}

/**
  * @brief  翻转指定LED状态
  * @param  led: LED编号 (1, 2, 3)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_Toggle(uint8_t led)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
      return 0;
    case 2:
      HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);
      return 0;
    case 3:
      HAL_GPIO_TogglePin(LED3_PORT, LED3_PIN);
      return 0;
    default:
      return -1;  /* 参数错误 */
  }
}

/**
  * @brief  设置LED为特定状态
  * @param  led: LED编号 (1, 2, 3)
  * @param  state: GPIO_PIN_SET(点亮) 或 GPIO_PIN_RESET(熄灭)
  * @retval 0=成功, 其他值=参数错误
  */
int LED_SetState(uint8_t led, GPIO_PinState state)
{
  switch(led)
  {
    case 1:
      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, state);
      return 0;
    case 2:
      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, state);
      return 0;
    case 3:
      HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, state);
      return 0;
    default:
      return -1;  /* 参数错误 */
  }
}

/**
  * @brief  获取LED当前状态
  * @param  led: LED编号 (1, 2, 3)
  * @retval GPIO_PIN_SET(点亮) 或 GPIO_PIN_RESET(熄灭)，错误时返回0xFF
  */
GPIO_PinState LED_GetState(uint8_t led)
{
  switch(led)
  {
    case 1:
      return HAL_GPIO_ReadPin(LED1_PORT, LED1_PIN);
    case 2:
      return HAL_GPIO_ReadPin(LED2_PORT, LED2_PIN);
    case 3:
      return HAL_GPIO_ReadPin(LED3_PORT, LED3_PIN);
    default:
      return (GPIO_PinState)0xFF;  /* 参数错误 */
  }
}

/**
  * @brief  全部LED点亮
  * @retval None
  */
void LED_AllOn(void)
{
  LED_On(1);
  LED_On(2);
  LED_On(3);
}

/**
  * @brief  全部LED熄灭
  * @retval None
  */
void LED_AllOff(void)
{
  LED_Off(1);
  LED_Off(2);
  LED_Off(3);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
