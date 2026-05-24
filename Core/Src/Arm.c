/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    Arm.c
	* @brief   机械臂初始化流程
	******************************************************************************
	*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Arm.h"
#include "Emm_V5.h"
#include "Limit.h"

/* 在RTOS运行时优先使用osDelay，避免阻塞Systick临界区；未启动RTOS时退回HAL_Delay */
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */

static void Arm_DelayMs(uint32_t delay_ms)
{
	if (delay_ms == 0u)
		return;

	if (osKernelGetState() == osKernelRunning)
	{
		osDelay(delay_ms);
	}
	else
	{
		HAL_Delay(delay_ms);
	}
}

#define ARM_HOME_POLL_MS               10u
#define ARM_HOME_RELEASE_TIMEOUT_MS    3000u

/**
	* @brief  反方向退出限位，直到限位释放
	* @param  motor_id: 电机ID
	* @param  limit_id: 限位开关ID
	* @param  release_dir: 退出限位方向
	* @param  vel: 退出速度
	* @param  acc: 加速度
	* @retval 0=成功, -1=错误或超时
	*/
static int Arm_ReleaseLimit(uint8_t motor_id, uint8_t limit_id,
							uint8_t release_dir, uint16_t vel, uint8_t acc)
{
	uint32_t elapsed_ms = 0u;
	int limit_state;

	limit_state = Limit_IsTriggered(limit_id);
	if (limit_state < 0)
		return -1;
	if (limit_state == 0)
		return 0;

	Emm_V5_Vel_Control(motor_id, release_dir, vel, acc, false);

	while (elapsed_ms < ARM_HOME_RELEASE_TIMEOUT_MS)
	{
		limit_state = Limit_IsTriggered(limit_id);
		if (limit_state < 0)
			break;
		if (limit_state == 0)
		{
			Emm_V5_Stop_Now(motor_id, false);
			Arm_DelayMs(20);
			return 0;
		}

		Arm_DelayMs(ARM_HOME_POLL_MS);
		elapsed_ms += ARM_HOME_POLL_MS;
	}

	Emm_V5_Stop_Now(motor_id, false);
	return -1;
}

/* USER CODE END 0 */

/**
	* @brief  机械臂初始化步骤1：7号电机运行至3号限位并停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_Step1(void)
{
	int limit_state;

	/* 7号电机(对应3号限位开关)，方向0，速度50，加速度30 
    * 方向0向左
    * */
	limit_state = Limit_IsTriggered(LIMIT_ID_3);
	if (limit_state < 0)
		return -1;
	if (limit_state == 1)
	{
		/* 已触发限位，先反方向退回 */
		Emm_V5_Pos_Control(7, 1, 200, 20, 7200, true, false);
		osDelay(1500);
		while (Limit_IsTriggered(LIMIT_ID_3) == 1)
		{
			Arm_DelayMs(10);
		}
	}

	osDelay(10);
	Emm_V5_Vel_Control(7, 0, 100, 20, false);

	while (1)
	{
		limit_state = Limit_IsTriggered(LIMIT_ID_3);
		if (limit_state == 1)
		{
			Emm_V5_Stop_Now(7, false);
			return 0;
		}

		if (limit_state < 0)
			return -1;

		Arm_DelayMs(10);
	}
}

/**
	* @brief  机械臂初始化步骤2：6号电机运行至2号限位并停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_Step2(void)
{
	int limit_state;

	/* 6号电机(对应2号限位开关)，方向0，速度50，加速度30
     * 方向0向内
     * */
	limit_state = Limit_IsTriggered(LIMIT_ID_2);
	if (limit_state < 0)
		return -1;
	if (limit_state == 1)
	{
		/* 已触发限位，先反方向退回 */
		Emm_V5_Pos_Control(6, 1, 200, 20, 7200, true, false);
		osDelay(1500);
		while (Limit_IsTriggered(LIMIT_ID_2) == 1)
		{
			Arm_DelayMs(10);
		}
	}

	osDelay(10);
	Emm_V5_Vel_Control(6, 0, 50, 30, false);

	while (1)
	{
		limit_state = Limit_IsTriggered(LIMIT_ID_2);
		if (limit_state == 1)
		{
			Emm_V5_Stop_Now(6, false);
			return 0;
		}

		if (limit_state < 0)
			return -1;

		Arm_DelayMs(10);
	}
}

/**
	* @brief  机械臂初始化步骤3：5号电机运行至1号限位并停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_Step3(void)
{
	int limit_state;

	/* 5号电机(对应1号限位开关)，方向1，速度50，加速度30
     * 方向1向下
     * */
	limit_state = Limit_IsTriggered(LIMIT_ID_1);
	if (limit_state < 0)
		return -1;
	if (limit_state == 1)
	{
		/* 已触发限位，先反方向退回 */
		Emm_V5_Pos_Control(5, 0, 200, 20, 7200, true, false);
		osDelay(1500);
		while (Limit_IsTriggered(LIMIT_ID_1) == 1)
		{
			Arm_DelayMs(10);
		}
	}

	osDelay(10);
	Emm_V5_Vel_Control(5, 1, 50, 30, false);

	while (1)
	{
		limit_state = Limit_IsTriggered(LIMIT_ID_1);
		if (limit_state == 1)
		{
			Emm_V5_Stop_Now(5, false);
			return 0;
		}

		if (limit_state < 0)
			return -1;

		Arm_DelayMs(10);
	}
}

/**
	* @brief  机械臂并行初始化：三路电机同时寻限并各自停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_AllParallel(void)
{
	int limit_state;
	uint8_t done_1 = 0u;
	uint8_t done_3 = 0u;
	uint8_t need_release_1 = 0u;
	uint8_t need_release_3 = 0u;
	uint8_t started_1 = 0u;
	uint8_t started_3 = 0u;

	/* Home Y first: motor 6 / limit 2. */
	limit_state = Limit_IsTriggered(LIMIT_ID_2);
	if (limit_state < 0)
		return -1;
	if (limit_state == 1)
	{
		if (Arm_ReleaseLimit(6, LIMIT_ID_2, 1, 100, 20) != 0)
			return -1;
	}

	Emm_V5_Vel_Control(6, 0, 100, 20, false);
	osDelay(10);

	while (1)
	{
		limit_state = Limit_IsTriggered(LIMIT_ID_2);
		if (limit_state < 0)
			goto arm_init_err;
		if (limit_state == 1)
		{
			Emm_V5_Stop_Now(6, false);
			osDelay(10);
			break;
		}

		osDelay(10);
	}

	/* Then home X and Z in parallel. */
	limit_state = Limit_IsTriggered(LIMIT_ID_3);
	if (limit_state < 0)
		goto arm_init_err;
	if (limit_state == 1)
	{
		if (Arm_ReleaseLimit(7, LIMIT_ID_3, 1, 120, 20) != 0)
			goto arm_init_err;
		need_release_3 = 1u;
	}

	limit_state = Limit_IsTriggered(LIMIT_ID_1);
	if (limit_state < 0)
		goto arm_init_err;
	if (limit_state == 1)
	{
		if (Arm_ReleaseLimit(5, LIMIT_ID_1, 0, 150, 20) != 0)
			goto arm_init_err;
		need_release_1 = 1u;
	}

	if (need_release_3 == 0u)
	{
		Emm_V5_Vel_Control(7, 0, 120, 20, false);
		osDelay(10);
		started_3 = 1u;
	}
	if (need_release_1 == 0u)
	{
		Emm_V5_Vel_Control(5, 1, 150, 20, false);
		osDelay(10);
		started_1 = 1u;
	}

	while ((done_1 == 0u) || (done_3 == 0u))
	{
		if (done_3 == 0u)
		{
			if (need_release_3 != 0u)
			{
				limit_state = Limit_IsTriggered(LIMIT_ID_3);
				if (limit_state < 0)
					goto arm_init_err;
				if (limit_state == 0)
				{
					Emm_V5_Vel_Control(7, 0, 120, 20, false);
					osDelay(10);
					need_release_3 = 0u;
					started_3 = 1u;
				}
			}
			else if (started_3 != 0u)
			{
				limit_state = Limit_IsTriggered(LIMIT_ID_3);
				if (limit_state < 0)
					goto arm_init_err;
				if (limit_state == 1)
				{
					Emm_V5_Stop_Now(7, false);
					osDelay(10);
					done_3 = 1u;
				}
			}
		}

		if (done_1 == 0u)
		{
			if (need_release_1 != 0u)
			{
				limit_state = Limit_IsTriggered(LIMIT_ID_1);
				if (limit_state < 0)
					goto arm_init_err;
				if (limit_state == 0)
				{
					Emm_V5_Vel_Control(5, 1, 150, 20, false);
					osDelay(10);
					need_release_1 = 0u;
					started_1 = 1u;
				}
			}
			else if (started_1 != 0u)
			{
				limit_state = Limit_IsTriggered(LIMIT_ID_1);
				if (limit_state < 0)
					goto arm_init_err;
				if (limit_state == 1)
				{
					Emm_V5_Stop_Now(5, false);
					osDelay(10);
					done_1 = 1u;
				}
			}
		}

		osDelay(10);
	}

	/* Release Y first, then X/Z, and set the released position to zero. */
	if (Arm_ReleaseLimit(6, LIMIT_ID_2, 1, 100, 20) != 0)
		goto arm_init_err;
	if (Arm_ReleaseLimit(7, LIMIT_ID_3, 1, 120, 20) != 0)
		goto arm_init_err;
	if (Arm_ReleaseLimit(5, LIMIT_ID_1, 0, 150, 20) != 0)
		goto arm_init_err;

	// Emm_V5_Origin_Set_O(7, true);
	// Emm_V5_Origin_Set_O(6, true);
	// Emm_V5_Origin_Set_O(5, true);
	Emm_V5_Reset_CurPos_To_Zero(6);
	Emm_V5_Reset_CurPos_To_Zero(7);
	Emm_V5_Reset_CurPos_To_Zero(5);

    // Move to reference position after homing if needed.
	// Emm_V5_Pos_Control(7, 1, 200, 20, 3200*2, true, false);
	// Emm_V5_Pos_Control(6, 1, 200, 20, 3200*4, true, false);
	// Emm_V5_Pos_Control(5, 0, 200, 20, 3200*12, true, false);
	return 0;

arm_init_err:
	Emm_V5_Stop_Now(6, false);
	osDelay(10);
	Emm_V5_Stop_Now(7, false);
	osDelay(10);
	Emm_V5_Stop_Now(5, false);
	return -1;
}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
