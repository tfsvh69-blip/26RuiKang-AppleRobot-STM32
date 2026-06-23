#ifndef __FRUIT_ACTUATOR_H__
#define __FRUIT_ACTUATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
  * @brief 树莓派返回的果子类型，用于后续分类投放。
  */
typedef enum
{
  FRUIT_BIG = 0,                  /* 大果，约 85 mm。 */
  FRUIT_SMALL = 1,                /* 小果，约 50 mm。 */
  FRUIT_TYPE_BIG = FRUIT_BIG,     /* 协议层兼容命名：大果。 */
  FRUIT_TYPE_SMALL = FRUIT_SMALL, /* 协议层兼容命名：小果。 */
  FRUIT_TYPE_UNKNOWN = 2          /* 未知大小，不抓取。 */
} FruitType_t;

/* 执行器接口通用返回值。 */
#define FRUIT_ACTUATOR_OK             0
#define FRUIT_ACTUATOR_ERR_PARAM     -1
#define FRUIT_ACTUATOR_ERR_SERVO     -2

/**
  * @brief 打开夹爪到待抓取/释放角度。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Gripper_Open(void);

/**
  * @brief 闭合夹爪，用于夹住一个苹果。
  * @note  TODO: 如实物需要区分大果/小果夹紧角度，后续再拆分接口或参数。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Gripper_Close(void);

/**
  * @brief 打开/复位剪刀，准备靠近苹果。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Cutter_Open(void);

/**
  * @brief 驱动剪刀到剪断角度。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Cutter_Cut(void);

/**
  * @brief 分类舵机回到中间安全位置。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Sorter_ToCenter(void);

/**
  * @brief 分类舵机转向当前大果暂存区。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Sorter_ToBigSide(void);

/**
  * @brief 分类舵机转向当前小果暂存区。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Sorter_ToSmallSide(void);

/**
  * @brief 根据果子类型完成分类投放。
  * @param type FRUIT_BIG 或 FRUIT_SMALL。
  * @note  动作顺序：分类舵机转向对应仓位，夹爪打开，分类舵机回中。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int Sorter_DropByType(FruitType_t type);

/**
  * @brief 大果暂存篮执行两次倒出动作，并最终回到装载状态。
  * @note  当前大小果暂存区已互换；大果使用原小果篮 SERVO_5：
  *        125 度为装载，170 度为倒出。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int FruitBasket_DumpBig(void);

/**
  * @brief 小果暂存篮执行三次倒出动作，并最终回到装载状态。
  * @note  当前大小果暂存区已互换；小果使用原大果篮 SERVO_4：
  *        135 度为装载，65 度为倒出。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int FruitBasket_DumpSmall(void);

/**
  * @brief 依次倒出当前大果暂存篮和当前小果暂存篮。
  * @retval FRUIT_ACTUATOR_OK 或负数错误码。
  */
int FruitBasket_DumpAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __FRUIT_ACTUATOR_H__ */
