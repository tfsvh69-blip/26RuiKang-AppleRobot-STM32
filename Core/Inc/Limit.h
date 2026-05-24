/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    Limit.h
	* @brief   限位开关模块 - 提供机械臂导轨限位的模块化接口
	******************************************************************************
	*/
/* USER CODE END Header */

#ifndef __LIMIT_H__
#define __LIMIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ----------------------------- Types & Consts ----------------------------- */

/**
	* @brief 限位开关编号（对应硬件：LIMIT1~LIMIT3）
	*/
typedef enum
{
	LIMIT_ID_1 = 1,
	LIMIT_ID_2 = 2,
	LIMIT_ID_3 = 3,
} Limit_Id_t;

#define LIMIT_COUNT 3u

/* 触发电平：1=低电平触发(常闭/下拉)，0=高电平触发 */
#define LIMIT_ACTIVE_LOW 1u

/* 限位开关定义 */
#define LIMIT1_PORT   GPIOE
#define LIMIT1_PIN    GPIO_PIN_15

#define LIMIT2_PORT   GPIOE
#define LIMIT2_PIN    GPIO_PIN_8

#define LIMIT3_PORT   GPIOE
#define LIMIT3_PIN    GPIO_PIN_7

/**
	* @brief  初始化限位开关模块
	* @retval None
	*/
void Limit_Init(void);

/**
	* @brief  读取限位开关GPIO原始电平
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval GPIO_PIN_SET 或 GPIO_PIN_RESET，错误时返回0xFF
	*/
GPIO_PinState Limit_ReadRaw(uint8_t limit);

/**
	* @brief  读取限位开关触发状态
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval 1=触发, 0=未触发, -1=参数错误
	*/
int Limit_IsTriggered(uint8_t limit);

/**
	* @brief  获取所有限位状态
	* @param  states: 长度至少为3的数组，存储3个限位的触发状态
	* @retval 0=成功, -1=参数错误
	*/
int Limit_ReadAll(uint8_t *states);

/**
	* @brief  检测限位触发事件（仅返回一次）
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval 1=检测到触发, 0=未触发, -1=错误
	*/
int Limit_IsTriggeredEdge(uint8_t limit);

/**
	* @brief  检测限位释放事件（仅返回一次）
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval 1=检测到释放, 0=未释放, -1=错误
	*/
int Limit_IsReleasedEdge(uint8_t limit);

#ifdef __cplusplus
}
#endif

#endif /* __LIMIT_H__ */
