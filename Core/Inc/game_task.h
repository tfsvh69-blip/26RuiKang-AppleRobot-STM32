#ifndef __GAME_TASK_H__
#define __GAME_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pi_protocol.h"
#include <stdint.h>

/**
  * @brief  慢速保持航向巡航，并配合树莓派连续识别完成多次停车抓取。
  * @param  tree_id 当前果树编号，原样传给树莓派 WATCH/SCAN 协议。
  * @param  view_id 当前观察方向，TREE_VIEW_LEFT 或 TREE_VIEW_RIGHT。
  * @param  total_distance_cm 本轮巡航的累计目标距离，单位 cm；多次被 HIT 打断后会继续累加，
  *                           直到总前进距离达到该值才退出。
  * @param  speed_rrp 底盘巡航速度，单位 rrp；正数表示前进。
  * @param  yaw_abs 需要保持的场地绝对航向角，单位度。
  * @param  kp 保持航向 PD 控制的比例系数。
  * @param  kd 保持航向 PD 控制的微分系数。
  * @note   流程：
  *         1. 调用 Base_ForwardDistanceCmHoldYawWatchPi() 慢速前进剩余距离；
  *         2. 如果 Pi 在 WATCH 模式下发送合法 HIT，底盘函数会立即停车并返回本次实际前进距离；
  *         3. 停稳后重新发送 SCAN，请 Pi 重新拍照并返回最终 FRUIT 坐标；
  *         4. 收到 FRUIT 后直接执行 FruitPick_PickOne()，包含夹爪、剪刀和分类投放完整流程；
  *         5. 抓取完成或 SCAN 返回 NONE 后，继续按剩余距离巡航。
  * @note   HIT 只用于停车触发，不作为最终抓取坐标；最终抓取只使用停稳后的 SCAN/FRUIT。
  * @retval 0 本轮累计距离走完，或正常结束。
  * @retval 负数 底盘、通信或抓取流程返回的错误码。
  */
int Game_CreepWatchAndPick(uint8_t tree_id,
                           TreeViewId_t view_id,
                           float total_distance_cm,
                           int16_t speed_rrp,
                           float yaw_abs,
                           float kp,
                           float kd);

#ifdef __cplusplus
}
#endif

#endif /* __GAME_TASK_H__ */
