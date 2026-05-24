/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    oled.h
  * @brief   SSD1306 OLED显示模块 - 基于I2C1(PB8/PB9) + TX DMA
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __OLED_H__
#define __OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* OLED参数（默认SSD1306 128x64） */
#define OLED_WIDTH              128u
#define OLED_HEIGHT             64u

/* 基于8x16字体的常用行列（兼容现有接口：line=1~4, column=1~16） */
#define OLED_TEXT_LINES         4u
#define OLED_TEXT_COLUMNS       16u

/* 常用I2C地址（7-bit：0x3C），HAL里需要左移1位 */
#define OLED_I2C_ADDR_7BIT      0x3Cu
#define OLED_I2C_ADDR           (OLED_I2C_ADDR_7BIT << 1)

/* API ----------------------------------------------------------------------*/

/**
  * @brief  初始化OLED（发送SSD1306初始化命令，并清屏）
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_Init(void);

/**
  * @brief  刷新OLED（把内部显存buffer写入屏幕）
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_Update(void);

/**
  * @brief  刷新OLED（OLED_Update的别名，语义更直观）
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_Flush(void);

/**
  * @brief  清屏（清空buffer并刷新）
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_Clear(void);

/**
  * @brief  清空内部显存buffer（不刷新屏幕）
  * @retval HAL状态（当前实现恒为HAL_OK）
  */
HAL_StatusTypeDef OLED_ClearBuffer(void);

/**
  * @brief  在buffer上绘制字符串（8x16字体，不立即刷新）
  * @param  line: 行(1~4)
  * @param  column: 列(1~16)
  * @param  str: ASCII字符串（当前内置字库主要覆盖数字/大写字母/空格，未覆盖字符将显示为空白）
  * @retval HAL状态
  * @note   调用后需要执行OLED_Flush/OLED_Update才会显示到屏幕
  */
HAL_StatusTypeDef OLED_DrawString(uint8_t line, uint8_t column, const char *str);

/**
  * @brief  在buffer上绘制无符号十进制数字（不立即刷新）
  * @param  line: 行(1~4)
  * @param  column: 列(1~16)
  * @param  num: 数字
  * @param  len: 显示长度(1~10)，左侧补空格
  * @retval HAL状态
  * @note   调用后需要执行OLED_Flush/OLED_Update才会显示到屏幕
  */
HAL_StatusTypeDef OLED_DrawNum(uint8_t line, uint8_t column, uint32_t num, uint8_t len);

/**
  * @brief  在buffer上绘制字符串（8x16字体），并立即刷新
  * @param  line: 行(1~4)
  * @param  column: 列(1~16)
  * @param  str: ASCII字符串（当前内置字库主要覆盖数字/大写字母/空格，未覆盖字符将显示为空白）
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_ShowString(uint8_t line, uint8_t column, const char *str);

/**
  * @brief  显示无符号十进制数字，并立即刷新
  * @param  line: 行(1~4)
  * @param  column: 列(1~16)
  * @param  num: 数字
  * @param  len: 显示长度(1~10)，左侧补空格
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_ShowNum(uint8_t line, uint8_t column, uint32_t num, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H__ */
