/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    Arm.h
	* @brief   机械臂初始化流程接口
	******************************************************************************
	*/
/* USER CODE END Header */

#ifndef __ARM_H__
#define __ARM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
	* @brief  机械臂初始化步骤1：7号电机运行至3号限位并停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_Step1(void);

/**
	* @brief  机械臂初始化步骤2：6号电机运行至2号限位并停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_Step2(void);

/**
	* @brief  机械臂初始化步骤3：5号电机运行至1号限位并停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_Step3(void);

/**
	* @brief  机械臂并行初始化：三路电机同时寻限并各自停止
	* @retval 0=成功, -1=错误
	*/
int Arm_Init_AllParallel(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_H__ */
