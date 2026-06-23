#ifndef __GAME_TASK_H__
#define __GAME_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pi_protocol.h"
#include <stdint.h>

/**
  * @brief  初始化比赛业务 UI 状态同步对象。
  * @note   当前内部使用 FreeRTOS EventGroup，让业务任务把状态传给 OLED 任务。
  */
void GameUi_Init(void);

/**
  * @brief  获取 Pi 启动等待流程需要显示在 OLED 第 4 行的短文本。
  * @retval 非 NULL 当前应显示的 16 字符以内 ASCII 文本。
  * @retval NULL 当前没有 Pi 状态需要显示。
  */
const char *GameUi_GetPiStatusText(void);

extern volatile int g_game_creep_last_scan_status;
extern volatile uint16_t g_game_creep_last_scan_seq;
extern volatile uint16_t g_game_creep_scan_timeout_count;
extern volatile PiNoneReason_t g_game_creep_last_none_reason;

/**
  * @brief  等待树莓派串口协议可用，并等待人工按键确认后才继续比赛主流程。
  * @note   循环发送 PING，收到 PONG 后等待 KEY1/PE3 带消抖按下。
  *         本函数不直接刷新 OLED，只更新 GameUi 状态，由 OLED 任务统一显示。
  */
void Game_WaitPiReadyAndUserStart(void);

/**
  * @brief  慢速保持航向巡航，并配合树莓派连续识别完成多次停车抓取。
  * @param  tree_id 当前果树编号，原样传给树莓派 WATCH/SCAN 协议。
  * @param  view_id 当前观察方向，TREE_VIEW_LEFT 或 TREE_VIEW_RIGHT。
  * @param  total_distance_cm 本轮巡航的累计目标距离，单位 cm；多次被 HIT 打断后会继续累加，
  *                           直到总前进距离达到该值才退出。
  * @param  speed_rrp 底盘巡航速度，单位 rrp；正数表示前进。
  * @param  yaw_abs 需要保持的相对本次 MCU 复位零点的航向角，单位度。
  * @param  kp 保持航向 PD 控制的比例系数。
  * @param  kd 保持航向 PD 控制的微分系数。
  * @note   流程：
  *         1. 调用 Base_ForwardDistanceCmHoldYawWatchPi() 慢速前进剩余距离；
  *         2. 如果 Pi 在 WATCH 模式下发送合法 HIT，底盘函数会立即停车并返回本次实际前进距离；
  *         3. 停稳后重新发送 SCAN，请 Pi 重新拍照并返回最终 FRUIT 坐标；
  *         4. 收到 FRUIT 后直接执行 FruitPick_PickOne()，包含夹爪、剪刀和分类投放完整流程；
  *         5. 抓取完成、SCAN 返回 NONE 或 SCAN 超时后，继续按剩余距离巡航。
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

/**
  * @brief  慢速保持航向巡航并监听 Pi，直到正前方距离达到阈值后正常退出。
  * @param  tree_id 当前果树编号，原样传给树莓派 WATCH/SCAN 协议。
  * @param  view_id 当前观察方向，TREE_VIEW_LEFT 或 TREE_VIEW_RIGHT。
  * @param  stop_dist_mm 正前方雷达停止距离，dist1 小于等于该值时函数返回 0。
  * @param  speed_rrp 底盘巡航速度，单位 rrp，正数表示前进。
  * @param  yaw_abs 需要保持的相对本次 MCU 复位零点的航向角，单位度。
  * @param  kp 保持航向 PD 控制的比例系数。
  * @param  kd 保持航向 PD 控制的微分系数。
  * @note   HIT 只用于停车触发；最终抓取只使用停车稳定后 SCAN 返回的 FRUIT。
  * @retval 0 前方距离阈值到达，正常结束。
  * @retval 负数 底盘、通信或抓取流程返回的错误码。
  */
int Game_CreepWatchAndPickUntilFrontDistance(uint8_t tree_id,
                                             TreeViewId_t view_id,
                                             uint16_t stop_dist_mm,
                                             int16_t speed_rrp,
                                             float yaw_abs,
                                             float kp,
                                             float kd,
                                             uint32_t min_run_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* __GAME_TASK_H__ */
