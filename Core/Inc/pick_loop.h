#ifndef __PICK_LOOP_H__
#define __PICK_LOOP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "fruit_pick.h"

#define PICK_LOOP_OK                  0
#define PICK_LOOP_ERR_PARAM          -1
#define PICK_LOOP_ERR_NO_FRUIT       -2
#define PICK_LOOP_ERR_PICK           -3

/**
  * @brief 连续抓取循环用于请求下一个最优苹果的回调函数。
  * @param out 输出目标；回调返回 0 时必须填充有效目标。
  * @param context 用户上下文，通常用于传入 pi_protocol、树编号、视角等状态。
  * @retval 0 表示获得有效目标；非 0 表示无果或请求错误，本轮循环停止。
  */
typedef int (*PickLoop_RequestFruitFn)(FruitTarget_t *out, void *context);

/**
  * @brief 当前固定观察点连续抓取配置。
  * @note  本模块不控制底盘，只重复执行：请求一个目标，抓取一个苹果。
  */
typedef struct
{
  PickLoop_RequestFruitFn request_fruit;  /* 获取单个目标的函数。 */
  void *context;                          /* 传给 request_fruit() 的用户上下文。 */
  uint8_t max_pick_count;                 /* 当前观察点最多抓取数量。 */
  uint32_t retry_delay_ms;                /* 每次成功抓取后的等待时间。 */
} PickLoopConfig_t;

/**
  * @brief 在当前观察点连续抓取多个苹果。
  * @param config 循环配置。
  * @param picked_count 可选输出，记录已成功抓取数量。
  * @retval PICK_LOOP_OK 或负数错误码。
  */
int PickLoop_RunCurrentView(const PickLoopConfig_t *config, uint8_t *picked_count);

#ifdef __cplusplus
}
#endif

#endif /* __PICK_LOOP_H__ */
