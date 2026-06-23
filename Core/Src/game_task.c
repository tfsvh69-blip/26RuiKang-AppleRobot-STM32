#include "game_task.h"
#include "base_control.h"
#include "button.h"
#include "fruit_pick.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include <stddef.h>

/*
 * SCAN 最大等待时间，单位 ms。
 * 用在 HIT 停车后请求树莓派重新取帧、推理并返回 FRUIT/NONE 的阶段。
 * 这个值只是超时上限；Pi 提前返回时不会等满。调小会更快退出，但可能误判 Pi 推理超时。
 */
#define GAME_PI_SCAN_TIMEOUT_MS       (10000u)

/*
 * HIT 后停车稳定等待时间，单位 ms。
 * 小车收到 Pi 的 HIT 后会先停车，再等待这段时间，最后发送 SCAN 重拍。
 * 调小可以更快进入 SCAN；太小可能车还没停稳，导致重拍坐标偏。
 */
#define GAME_CREEP_STOP_SETTLE_MS     (300u)

/*
 * 单果抓取、剪切、投放完成后的恢复等待时间，单位 ms。
 * 等待结束后才继续巡航找下一个果。
 * 调小可以明显提升连续采摘速度；太小可能机械臂或果仓还在晃动。
 */
#define GAME_CREEP_RESUME_DELAY_MS    (2800u)

/*
 * 赛前/流程开始前 PING 树莓派的单次等待时间，单位 ms。
 * 只影响等待 Pi 就绪阶段，不影响采摘中的 WATCH/SCAN 速度。
 */
#define GAME_PI_READY_PING_TIMEOUT_MS (50u)

/*
 * Pi 未就绪时，两次 PING 之间的重试间隔，单位 ms。
 * 调小会更频繁查询 Pi；一般不需要频繁修改。
 */
#define GAME_PI_READY_RETRY_DELAY_MS  (100u)

#define GAME_UI_EVENT_PI_WAITING      (1u << 0)
#define GAME_UI_EVENT_PI_READY        (1u << 1)
#define GAME_UI_EVENT_PI_START        (1u << 2)
#define GAME_UI_EVENT_PI_MASK         (GAME_UI_EVENT_PI_WAITING | GAME_UI_EVENT_PI_READY | GAME_UI_EVENT_PI_START)

static EventGroupHandle_t s_gameUiEventGroup = NULL;

volatile int g_game_creep_last_scan_status = 0;
volatile uint16_t g_game_creep_last_scan_seq = 0u;
volatile uint16_t g_game_creep_scan_timeout_count = 0u;
volatile PiNoneReason_t g_game_creep_last_none_reason = PI_NONE_REASON_UNKNOWN;

static void GameUi_SetPiStatus(EventBits_t status_bit)
{
  if (s_gameUiEventGroup == NULL)
  {
    return;
  }

  (void)xEventGroupClearBits(s_gameUiEventGroup, GAME_UI_EVENT_PI_MASK);
  (void)xEventGroupSetBits(s_gameUiEventGroup, status_bit);
}

void GameUi_Init(void)
{
  if (s_gameUiEventGroup == NULL)
  {
    s_gameUiEventGroup = xEventGroupCreate();
  }
}

const char *GameUi_GetPiStatusText(void)
{
  EventBits_t ui_bits;

  if (s_gameUiEventGroup == NULL)
  {
    return NULL;
  }

  ui_bits = xEventGroupGetBits(s_gameUiEventGroup);
  if ((ui_bits & GAME_UI_EVENT_PI_START) != 0u)
  {
    return "PI START        ";
  }
  if ((ui_bits & GAME_UI_EVENT_PI_READY) != 0u)
  {
    return "PI READY K1     ";
  }
  if ((ui_bits & GAME_UI_EVENT_PI_WAITING) != 0u)
  {
    return "PI WAIT         ";
  }

  return NULL;
}

void Game_WaitPiReadyAndUserStart(void)
{
  uint16_t seq = 1u;
  int pi_status;

  GameUi_SetPiStatus(GAME_UI_EVENT_PI_WAITING);

  for (;;)
  {
    pi_status = Pi_Ping(seq, GAME_PI_READY_PING_TIMEOUT_MS);
    seq++;

    if (pi_status == PI_OK)
    {
      break;
    }

    osDelay(GAME_PI_READY_RETRY_DELAY_MS);
  }

  GameUi_SetPiStatus(GAME_UI_EVENT_PI_READY);

  while (Button_ReadDebounce((uint8_t)BUTTON_KEY1, 30u) == KEY_PRESSED)
  {
    osDelay(20u);
  }

  while (Button_ReadDebounce((uint8_t)BUTTON_KEY1, 30u) != KEY_PRESSED)
  {
    osDelay(20u);
  }

  GameUi_SetPiStatus(GAME_UI_EVENT_PI_START);
}

/**
  * @brief  慢速巡航并等待树莓派发现可抓苹果，支持多次停车抓取。
  * @param  tree_id 当前果树编号，原样发送给树莓派 WATCH/SCAN 协议。
  * @param  view_id 当前观察方向，TREE_VIEW_LEFT 表示左侧观察，TREE_VIEW_RIGHT 表示右侧观察。
  * @param  total_distance_cm 本轮巡航允许累计前进的总距离，单位 cm。
  *         函数内部会记录已经走过的距离；即使中途多次被 Pi 的 HIT 打断，
  *         抓取后也会继续按剩余距离前进，直到累计距离达到该值才退出。
  * @param  speed_rrp 底盘巡航速度，单位 rrp；正数表示前进，不能为 0。
  * @param  yaw_abs 巡航时需要保持的相对本次 MCU 复位零点的航向角，单位度。
  * @param  kp 保持航向 PD 控制的比例系数，传给底盘保持航向函数。
  * @param  kd 保持航向 PD 控制的微分系数，传给底盘保持航向函数。
  * @note   执行流程：
  *         1. 根据 total_distance_cm 和 traveled_cm 计算 remaining_cm；
  *         2. 调用 Base_ForwardDistanceCmHoldYawWatchPi() 慢速前进并持续监听 Pi 的 HIT；
  *         3. 如果走完整段剩余距离且没有 HIT，函数直接正常退出；
  *         4. 如果收到合法 HIT，底盘函数会停车并返回本次实际估算前进距离；
  *         5. 停稳后发送 SCAN，请 Pi 重新拍照并返回最终 FRUIT 坐标；
  *         6. 如果 SCAN 返回 NONE 或超时，不抓取，继续按剩余距离巡航；
  *         7. 如果 SCAN 返回 FRUIT，执行 FruitPick_PickOne() 完整抓取、剪切和分类投放；
  *         8. 抓取完成后额外等待一小段时间，确保 XYZ 已回到不会遮挡相机的位置，再继续巡航；
  *         9. 持续循环，直到累计距离达到 total_distance_cm。
  * @note   Pi 的 HIT 只用于触发停车，不能作为最终抓取坐标。
  *         真正抓取只使用停车稳定后 SCAN 返回的 FRUIT 坐标。
  * @retval 0 正常结束：累计距离走完，或最后一段未命中直接走完。
  * @retval -1 参数错误，例如 total_distance_cm <= 0 或 speed_rrp == 0。
  * @retval 其他负数 直接透传底盘、Pi 通信或 FruitPick_PickOne() 返回的错误码。
  */
int Game_CreepWatchAndPick(uint8_t tree_id,
                           TreeViewId_t view_id,
                           float total_distance_cm,
                           int16_t speed_rrp,
                           float yaw_abs,
                           float kp,
                           float kd)
{
  float traveled_cm = 0.0f;
  float step_traveled_cm;
  float remaining_cm;
  int move_status;
  int pi_status;
  int pick_status;
  uint16_t scan_seq = 0u;
  FruitTarget_t fruit = {FRUIT_BIG, 0, 0, 0, 0, 0};

  if ((total_distance_cm <= 0.0f) || (speed_rrp == 0))
  {
    return -1;
  }

  g_game_creep_last_scan_status = 0;
  g_game_creep_last_scan_seq = 0u;
  g_game_creep_scan_timeout_count = 0u;
  g_game_creep_last_none_reason = PI_NONE_REASON_UNKNOWN;

  while (traveled_cm < total_distance_cm)
  {
    remaining_cm = total_distance_cm - traveled_cm;
    step_traveled_cm = 0.0f;

    move_status = Base_ForwardDistanceCmHoldYawWatchPi(remaining_cm,
                                                       speed_rrp,
                                                       yaw_abs,
                                                       kp,
                                                       kd,
                                                       tree_id,
                                                       view_id,
                                                       &step_traveled_cm);
    traveled_cm += step_traveled_cm;

    if (move_status == 0)
    {
      return 0;
    }

    if (move_status != 1)
    {
      return move_status;
    }

    osDelay(GAME_CREEP_STOP_SETTLE_MS);
    scan_seq = 0u;
    pi_status = Pi_RequestBestFruitWithSeq(tree_id,
                                           view_id,
                                           &fruit,
                                           GAME_PI_SCAN_TIMEOUT_MS,
                                           &scan_seq);
    g_game_creep_last_scan_seq = scan_seq;
    g_game_creep_last_scan_status = pi_status;
    if (pi_status == PI_ERR_NONE)
    {
      g_game_creep_last_none_reason = Pi_GetLastNoneReason();
      continue;
    }
    if (pi_status == PI_ERR_TIMEOUT)
    {
      g_game_creep_scan_timeout_count++;
      (void)Pi_CancelScan(scan_seq);
      continue;
    }
    if (pi_status != PI_OK)
    {
      return pi_status;
    }

    pick_status = FruitPick_PickOne(&fruit);
    if (pick_status != FRUIT_PICK_OK)
    {
      return pick_status;
    }

    /* 抓取流程返回后再额外等一小段，避免 XYZ 刚回收完就立即巡航，继续挡住相机视野。 */
    osDelay(GAME_CREEP_RESUME_DELAY_MS);
  }

  return 0;
}

/**
  * @brief  按前方雷达距离作为退出条件的连续巡航识别抓取流程。
  * @param  tree_id 当前果树编号，原样发送给树莓派 WATCH/SCAN 协议。
  * @param  view_id 当前观察方向，TREE_VIEW_LEFT 表示左侧观察，TREE_VIEW_RIGHT 表示右侧观察。
  * @param  stop_dist_mm 正前方雷达停止距离，dist1 小于等于该值时认为本段巡航结束。
  * @param  speed_rrp 底盘巡航速度，单位 rrp；正数表示前进，不能为 0。
  * @param  yaw_abs 巡航时需要保持的相对本次 MCU 复位零点的航向角，单位度。
  * @param  kp 保持航向 PD 控制的比例系数，传给底盘保持航向函数。
  * @param  kd 保持航向 PD 控制的微分系数，传给底盘保持航向函数。
  * @note   本函数和 Game_CreepWatchAndPick() 的抓取逻辑相同，区别是退出条件不同：
  *         1. 本函数不按累计前进距离退出，而是调用底盘函数持续前进到 dist1 <= stop_dist_mm；
  *         2. 前进过程中底盘函数会发送 WATCH，并持续轮询树莓派的 HIT；
  *         3. 如果前方距离先到阈值，底盘函数返回 0，本函数也返回 0，表示正常结束；
  *         4. 如果先收到合法 HIT，底盘会立即停车并返回 1；
  *         5. HIT 只用于停车触发，不作为最终抓取坐标；
  *         6. 停稳后必须重新发送 SCAN，让树莓派重拍并返回最终 FRUIT 坐标；
  *         7. SCAN 返回 NONE 或超时时，本次不抓取，继续巡航直到前方距离到达阈值；
  *         8. SCAN 返回 FRUIT 时，执行 FruitPick_PickOne() 完整抓取、剪切和分类投放；
  *         9. 抓取完成后等待一小段时间，再继续下一轮 WATCH 巡航。
  * @retval 0 前方距离阈值到达，正常结束。
  * @retval -1 参数错误，例如 speed_rrp == 0。
  * @retval 其他负数 直接透传底盘、Pi 通信或 FruitPick_PickOne() 返回的错误码。
  */
int Game_CreepWatchAndPickUntilFrontDistance(uint8_t tree_id,
                                             TreeViewId_t view_id,
                                             uint16_t stop_dist_mm,
                                             int16_t speed_rrp,
                                             float yaw_abs,
                                             float kp,
                                             float kd,
                                             uint32_t min_run_time_ms)
{
  int move_status;
  int pi_status;
  int pick_status;
  uint16_t scan_seq = 0u;
  FruitTarget_t fruit = {FRUIT_BIG, 0, 0, 0, 0, 0};

  if (speed_rrp == 0)
  {
    return -1;
  }

  g_game_creep_last_scan_status = 0;
  g_game_creep_last_scan_seq = 0u;
  g_game_creep_scan_timeout_count = 0u;
  g_game_creep_last_none_reason = PI_NONE_REASON_UNKNOWN;

  for (;;)
  {
    move_status = Base_ForwardUntilFrontDistanceHoldYawWatchPi(stop_dist_mm,
                                                               speed_rrp,
                                                               yaw_abs,
                                                               kp,
                                                               kd,
                                                               tree_id,
                                                               view_id,
                                                               min_run_time_ms);
    if (move_status == 0)
    {
      return 0;
    }

    if (move_status != 1)
    {
      return move_status;
    }

    osDelay(GAME_CREEP_STOP_SETTLE_MS);
    scan_seq = 0u;
    pi_status = Pi_RequestBestFruitWithSeq(tree_id,
                                           view_id,
                                           &fruit,
                                           GAME_PI_SCAN_TIMEOUT_MS,
                                           &scan_seq);
    g_game_creep_last_scan_seq = scan_seq;
    g_game_creep_last_scan_status = pi_status;
    if (pi_status == PI_ERR_NONE)
    {
      g_game_creep_last_none_reason = Pi_GetLastNoneReason();
      continue;
    }
    if (pi_status == PI_ERR_TIMEOUT)
    {
      g_game_creep_scan_timeout_count++;
      (void)Pi_CancelScan(scan_seq);
      continue;
    }
    if (pi_status != PI_OK)
    {
      return pi_status;
    }

    pick_status = FruitPick_PickOne(&fruit);
    if (pick_status != FRUIT_PICK_OK)
    {
      return pick_status;
    }

    osDelay(GAME_CREEP_RESUME_DELAY_MS);
  }
}
