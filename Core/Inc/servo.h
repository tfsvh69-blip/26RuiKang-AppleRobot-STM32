/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    servo.h
  * @brief   舵机控制模块接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ----------------------------- Types & Consts ----------------------------- */

/**
  * @brief 舵机ID枚举（数组索引从0开始，便于统一管理）
  */
typedef enum
{
  SERVO_1 = 0,
  SERVO_2,
  SERVO_3,
  SERVO_4,
  SERVO_5,
  SERVO_6,
  SERVO_7,
  SERVO_8,
  SERVO_COUNT
} servo_id_e;


/* 舵机安全角度边界（用于批量动作，防止机械冲击） */
#define kSafeMinAngle  0.0f
#define kSafeMaxAngle 540.0f
/* 舵机脉宽与角度边界定义 */
#define SERVO_PULSE_MIN_US      500u
#define SERVO_PULSE_MID_US      1500u
#define SERVO_PULSE_MAX_US      2500u

#define SERVO_ANGLE_MIN_DEG     0.0f
#define SERVO_ANGLE_MAX_DEG     270.0f

/**
  * @brief  舵机统一初始化
  * @note   启动8路PWM输出，并将初始脉宽设置为1500us
  * @retval None
  */
void Servo_Init(void);

/**
  * @brief  设置指定舵机脉宽（单位us）
  * @param  id: 舵机ID，取值SERVO_1~SERVO_8
  * @param  pulse_us: 目标脉宽，允许范围500~2500us
  * @retval 0=成功, -1=参数错误
  */
int8_t Servo_SetPulse(servo_id_e id, uint16_t pulse_us);

/**
  * @brief  设置指定舵机角度（270度舵机）
  * @param  id: 舵机ID，取值SERVO_1~SERVO_8
  * @param  angle: 目标角度，允许范围0.0~270.0度
  * @retval 0=成功, -1=参数错误
  */
int8_t Servo_SetAngle(servo_id_e id, float angle);

/* 配置好角度之后的初始化，将舵机设置为上电复位前的安全避让状态。 */
void Servo_SetAll_Init(void);

/**
  * @brief  大小果篮打到垂直避让状态。
  * @note   XYZ 上电回零前调用，避免果篮与机械结构干涉。
  */
void Servo_SetFruitBasketsVertical(void);

/**
  * @brief  大小果篮回到装载状态。
  * @note   XYZ 回零完成且 Y 轴移动到 40mm 后调用。
  */
void Servo_SetFruitBasketsLoad(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
