#ifndef __FRUIT_PICK_H__
#define __FRUIT_PICK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "fruit_actuator.h"

/**
  * @brief 单个苹果目标，坐标为机械臂绝对坐标。
  * @note  x/y/z 已由树莓派完成相机坐标到机械臂坐标映射；
  *        STM32 这里只做范围校验和动作执行。
  */
typedef struct
{
  FruitType_t type;       /* 果子类型：FRUIT_BIG 或 FRUIT_SMALL。 */
  int32_t x_mm;           /* 机械臂 X 轴绝对坐标，单位 mm。 */
  int32_t y_mm;           /* 机械臂 Y 轴绝对坐标，单位 mm。 */
  int32_t z_mm;           /* 机械臂 Z 轴绝对坐标，单位 mm。 */
  int32_t diameter_mm;    /* 视觉估算的果子真实直径，单位 mm。 */
  int32_t score;          /* 视觉置信度，建议范围 0-100。 */
} FruitTarget_t;

/* FruitPick_PickOne() 返回值。 */
#define FRUIT_PICK_OK                  0
#define FRUIT_PICK_ERR_PARAM          -1
#define FRUIT_PICK_ERR_UNREACHABLE    -2
#define FRUIT_PICK_ERR_PREMOVE        -3
#define FRUIT_PICK_ERR_GRIPMOVE       -4
#define FRUIT_PICK_ERR_RETRACT_Y      -5
#define FRUIT_PICK_ERR_DROP_MOVE      -6
#define FRUIT_PICK_ERR_DROP           -7
#define FRUIT_PICK_ERR_ACTUATOR       -8

/**
  * @brief 按树莓派返回的绝对坐标抓取一个苹果。
  * @param fruit 树莓派返回的单果目标。
  * @note  流程：打开夹爪/剪刀，直接移动到抓取点，
  *        等待到位稳定后夹紧，剪断，直接 Y 轴后撤到 40mm，
  *        移动到投放点，分类释放。
  *        不计算单独剪切点，剪断后不做 Z 轴上抬。
  * @retval FRUIT_PICK_OK 或负数错误码。
  */
int FruitPick_PickOne(const FruitTarget_t *fruit);

#ifdef __cplusplus
}
#endif

#endif /* __FRUIT_PICK_H__ */
