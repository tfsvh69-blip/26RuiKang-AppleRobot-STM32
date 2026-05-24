/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    Limit.c
	* @brief   限位开关模块实现
	******************************************************************************
	*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Limit.h"

/* USER CODE BEGIN 0 */

static uint8_t limit_last_state[3] = {0, 0, 0};

static GPIO_PinState Limit_ReadPin(uint8_t limit)
{
	switch(limit)
	{
		case 1:
			return HAL_GPIO_ReadPin(LIMIT1_PORT, LIMIT1_PIN);
		case 2:
			return HAL_GPIO_ReadPin(LIMIT2_PORT, LIMIT2_PIN);
		case 3:
			return HAL_GPIO_ReadPin(LIMIT3_PORT, LIMIT3_PIN);
		default:
			return (GPIO_PinState)0xFF;
	}
}

static int Limit_PinToTriggered(GPIO_PinState state)
{
#if (LIMIT_ACTIVE_LOW == 1u)
	return (state == GPIO_PIN_RESET) ? 1 : 0;
#else
	return (state == GPIO_PIN_SET) ? 1 : 0;
#endif
}

/* USER CODE END 0 */

/**
	* @brief  初始化限位开关模块
	* @retval None
	*/
void Limit_Init(void)
{
	/* 限位GPIO初始化由MX_GPIO_Init完成，此函数保留用于扩展 */
	limit_last_state[0] = (uint8_t)Limit_IsTriggered(1);
	limit_last_state[1] = (uint8_t)Limit_IsTriggered(2);
	limit_last_state[2] = (uint8_t)Limit_IsTriggered(3);
}

/**
	* @brief  读取限位开关GPIO原始电平
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval GPIO_PIN_SET 或 GPIO_PIN_RESET，错误时返回0xFF
	*/
GPIO_PinState Limit_ReadRaw(uint8_t limit)
{
	return Limit_ReadPin(limit);
}

/**
	* @brief  读取限位开关触发状态
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval 1=触发, 0=未触发, -1=参数错误
	*/
int Limit_IsTriggered(uint8_t limit)
{
	GPIO_PinState state;

	if(limit < 1 || limit > 3)
		return -1;

	state = Limit_ReadPin(limit);
	if(state == (GPIO_PinState)0xFF)
		return -1;

	return Limit_PinToTriggered(state);
}

/**
	* @brief  获取所有限位状态
	* @param  states: 长度至少为3的数组，存储3个限位的触发状态
	* @retval 0=成功, -1=参数错误
	*/
int Limit_ReadAll(uint8_t *states)
{
	int i;
	int st;

	if(states == NULL)
		return -1;

	for(i = 0; i < (int)LIMIT_COUNT; i++)
	{
		st = Limit_IsTriggered((uint8_t)(i + 1));
		if (st < 0)
			return -1;
		states[i] = (uint8_t)st;
	}

	return 0;
}

/**
	* @brief  检测限位触发事件（仅返回一次）
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval 1=检测到触发, 0=未触发, -1=错误
	*/
int Limit_IsTriggeredEdge(uint8_t limit)
{
	int current_state;
	int edge_detected = 0;

	if(limit < 1 || limit > 3)
		return -1;

	current_state = Limit_IsTriggered(limit);
	if(current_state < 0)
		return -1;

	if(current_state == 1 && limit_last_state[limit - 1] == 0)
	{
		edge_detected = 1;
	}

	limit_last_state[limit - 1] = (uint8_t)current_state;
	return edge_detected;
}

/**
	* @brief  检测限位释放事件（仅返回一次）
	* @param  limit: 限位编号 (1, 2, 3)
	* @retval 1=检测到释放, 0=未释放, -1=错误
	*/
int Limit_IsReleasedEdge(uint8_t limit)
{
	int current_state;
	int edge_detected = 0;

	if(limit < 1 || limit > 3)
		return -1;

	current_state = Limit_IsTriggered(limit);
	if(current_state < 0)
		return -1;

	if(current_state == 0 && limit_last_state[limit - 1] == 1)
	{
		edge_detected = 1;
	}

	limit_last_state[limit - 1] = (uint8_t)current_state;
	return edge_detected;
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
