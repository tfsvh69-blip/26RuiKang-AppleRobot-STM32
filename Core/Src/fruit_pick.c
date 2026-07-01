#include "fruit_pick.h"
#include "arm_motion.h"
#include "cmsis_os.h"

/*
 * 单果抓取动作参数。
 *
 * 抓取流程不再使用预备点：
 *   树莓派返回的 x/y/z 就是机械臂最终抓取点。
 *   机械臂直接移动到该点，等待稳定后夹爪，再剪刀剪一次。
 *   剪断后不抬 Z，直接把 Y 轴后撤到 40mm 安全线。
 *
 * 大果投放准备点：
 *   当前大小果暂存区已整体互换。
 *   抓到大苹果后，XYZ 先移动到原小果暂存区准备点 (220, 40, 250)，
 *   再让分类舵机转到当前大果暂存区方向。
 *
 * 小果投放准备点：
 *   当前大小果暂存区已整体互换。
 *   抓到小苹果后，XYZ 先移动到原大果暂存区准备点 (40, 40, 250)，
 *   再让分类舵机转到当前小果暂存区方向。
 */
#define FRUIT_PICK_BIG_DROP_X_MM             220.0f
#define FRUIT_PICK_BIG_DROP_Y_MM             40.0f
#define FRUIT_PICK_BIG_DROP_Z_MM             265.0f
#define FRUIT_PICK_SMALL_DROP_X_MM           40.0f
#define FRUIT_PICK_SMALL_DROP_Y_MM           40.0f
#define FRUIT_PICK_SMALL_DROP_Z_MM           265.0f

/*
 * 抓取流程使用的 XYZ 快速运动参数。
 * 当前 arm_motion 默认普通运动也是 800rpm / acc=80；
 * 这里保留独立宏，方便后续单独调整抓取流程速度。
 *
 * 注意：
 *   Z 轴机械换算是 3200 脉冲 = 10mm，同样 rpm 下线速度只有 X/Y 的 1/4，
 *   所以 Z 轴会天然比 X/Y 慢。这里提高 rpm 主要是为了改善 Z/Y 轴抓取流程速度。
 *
 * 调参建议：
 *   如果实车出现抖动、冲击、丢步或结构晃动，优先降到 600rpm / acc=60；
 *   如果动作稳定但仍嫌慢，再逐步增加，不要一次加太多。
 */
#define FRUIT_PICK_ARM_VEL_RPM               800u  /* 抓取流程中 XYZ 运动速度，单位 rpm；调大动作更快，但更容易冲击、抖动或丢步。 */
#define FRUIT_PICK_ARM_ACC                   150u  /* 抓取流程中 XYZ 加速度；调大起停更快，但机械晃动也会增大。 */


// ================== 抓取流程动作等待时间参数 ==================

/**
 * @brief X/Z 轴到位后、Y 轴开始伸出前的停稳等待时间，单位 ms。
 *
 * 使用位置：
 *   抓取移动第一步（只把 X/Z 移动到抓取点，Y 轴保持不伸出）之后，
 *   第二步（单独把 Y 轴伸到抓取点）之前。
 *
 * 作用：
 *   抓取移动被拆成“先对齐 X/Z、再伸 Y”两步，目的是让夹爪先在挂果绳
 *   正对面把水平横向 X 和高度 Z 对齐好，再平行地面伸出 Y，
 *   避免整个夹爪斜着冲出去把绳子撞出夹爪有效范围。
 *   由于 Emm_V5 还没有接入真实到位反馈，这里用固定等待确保 X/Z 完全停稳后再伸 Y。
 *
 * 调参建议：
 *   如果 X/Z 还在惯性晃动 Y 就伸出、导致对不准挂果绳，增大此值。
 *   如果 X/Z 停稳很快、想提升采摘速度，可小幅减小。
 */
#define FRUIT_PICK_XZ_ARRIVE_WAIT_MS         500u  /* X/Z 到位后、伸 Y 前的停稳等待；太小会没对齐就斜着伸出。 */

/**
 * @brief XYZ 到达苹果中心抓取点后的额外稳定等待时间，单位 ms。
 *
 * 使用位置：
 *   Arm_MoveToPoint() 移动到树莓派返回的苹果中心坐标之后，
 *   Gripper_Close() 夹爪闭合之前。
 *
 * 作用：
 *   arm_motion 内部已经会根据目标距离和速度估算 XYZ 运动等待时间，
 *   但 Emm_V5 当前还没有接入真实“运动完成”反馈，所以这里再额外等一段时间，
 *   让最后移动的轴，尤其是 Y 轴，完全停稳后再夹苹果。
 *
 * 调参建议：
 *   如果实车出现“还没停稳就夹”“夹偏”“夹空”，优先增大此值。
 *   如果动作已经稳定但整套流程偏慢，可以小幅减小此值。
 *   数值越大越稳，但会降低采摘速度。
 */
#define FRUIT_PICK_GRIP_ARRIVE_WAIT_MS       1800u /* XYZ 到达苹果中心后、夹爪闭合前的额外稳定等待；太小会没停稳就夹。 */

/**
 * @brief 夹爪闭合后的保持等待时间，单位 ms。
 *
 * 使用位置：
 *   Gripper_Close() 夹爪闭合之后，
 *   Cutter_Cut() 剪刀剪切之前。
 *
 * 作用：
 *   给 SERVO_2 夹爪舵机留出实际转动和机构压紧的时间，
 *   确保苹果已经被夹稳，再开始剪断挂果绳。
 *
 * 调参建议：
 *   如果苹果容易滑动、夹不牢，适当增大此值。
 *   如果夹爪动作很快且稳定，可以小幅减小此值。
 */
#define FRUIT_PICK_GRIP_WAIT_MS              300u  /* 夹爪闭合后、剪刀剪切前的等待；给夹爪压紧苹果留时间。 */

/**
 * @brief 剪刀剪切闭合后的等待时间，单位 ms。
 *
 * 使用位置：
 *   每次 Cutter_Cut() 之后，
 *   下一次 Cutter_Open() 之前。
 *
 * 作用：
 *   给 SERVO_1 剪刀舵机留出从张开位置转到剪断位置的时间，
 *   并让剪刀在闭合位置保持片刻，确保绳子被剪断。
 *   当前流程只执行一次“剪切-张开”。
 *
 * 调参建议：
 *   如果出现绳子没有剪断、只剪到一半，优先增大此值。
 *   如果剪刀动作已经可靠，可以在保证剪断的前提下小幅减小。
 */
#define FRUIT_PICK_CUT_WAIT_MS               1000u /* 剪刀闭合剪断后的保持时间；太小可能绳子没剪断。 */

/**
 * @brief 剪刀张开后的等待时间，单位 ms。
 *
 * 使用位置：
 *   每次 Cutter_Open() 之后，
 *   下一次 Cutter_Cut() 或 Y 轴后撤动作之前。
 *
 * 作用：
 *   确保 SERVO_1 剪刀完全回到张开角度。
 *   剪刀张开后必须保持张开状态，然后机械臂直接 Y 轴后撤到 40mm。
 *
 * 调参建议：
 *   如果剪刀还没完全张开就开始下一步动作，增大此值。
 *   如果张开动作已经很快且稳定，可以小幅减小。
 */
#define FRUIT_PICK_CUT_OPEN_WAIT_MS          100u  /* 剪刀重新张开后的等待；太小可能剪刀未完全打开就开始后撤。 */

/**
 * @brief Y 轴后撤到 40mm 安全线后的额外停稳等待时间，单位 ms。
 *
 * 当前 Emm_V5 还没有接入真实到位反馈，Arm_MoveToPoint() 内部只能按距离估算等待。
 * 后撤完成后先额外等一小段时间，再移动到投放准备点，避免 Y 轴还在惯性运动时开始后续动作。
 */
#define FRUIT_PICK_RETRACT_Y_SETTLE_WAIT_MS  500u /* Y 轴后撤到 40mm 安全线后的稳定等待；太小可能还在惯性晃动就去投放点。 */

/**
 * @brief XYZ 到达投放准备点后的额外停稳等待时间，单位 ms。
 *
 * 分类舵机转向果仓并释放夹爪前必须保证机械臂已经停稳，尤其是 Y 轴已经回到 40mm。
 */
#define FRUIT_PICK_DROP_ARRIVE_WAIT_MS       1000u /* XYZ 到达大小果投放准备点后的等待；等待稳定后再转分类舵机并松夹爪。 */

static void FruitPick_DelayMs(uint32_t delay_ms)
{
  if (delay_ms == 0u)
  {
    return;
  }

  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static ArmMotionStatus_t FruitPick_MoveToPointAbs(const ArmPoint_t *point)
{
  if (point == NULL)
  {
    return ARM_MOTION_ERR_PARAM;
  }

  /*
   * TODO: Replace with Arm_MoveToPointAbs(*point) after arm_motion is fully
   * migrated to the AGENTS.md absolute-position API.
   */
  return Arm_MoveToPoint(point->x_mm, point->y_mm, point->z_mm);
}

static ArmMotionStatus_t FruitPick_MoveYToAbs(float y_mm)
{
  ArmPoint_t current;
  ArmMotionStatus_t status;

  status = Arm_GetCurrentPosition(&current);
  if (status != ARM_MOTION_OK)
  {
    return status;
  }

  current.y_mm = y_mm;
  return FruitPick_MoveToPointAbs(&current);
}

static int FruitPick_GetDropPoint(FruitType_t type, ArmPoint_t *drop)
{
  if (drop == NULL)
  {
    return FRUIT_PICK_ERR_PARAM;
  }

  if (type == FRUIT_BIG)
  {
    drop->x_mm = FRUIT_PICK_BIG_DROP_X_MM;
    drop->y_mm = FRUIT_PICK_BIG_DROP_Y_MM;
    drop->z_mm = FRUIT_PICK_BIG_DROP_Z_MM;
    return FRUIT_PICK_OK;
  }

  if (type == FRUIT_SMALL)
  {
    drop->x_mm = FRUIT_PICK_SMALL_DROP_X_MM;
    drop->y_mm = FRUIT_PICK_SMALL_DROP_Y_MM;
    drop->z_mm = FRUIT_PICK_SMALL_DROP_Z_MM;
    return FRUIT_PICK_OK;
  }

  return FRUIT_PICK_ERR_PARAM;
}

static int FruitPick_CutOnce(void)
{
  int ret;

  /*
   * 剪刀动作顺序：
   *   剪一次 -> 张开。
   * 最终保持剪刀张开后，才允许 Y 轴开始后撤。
   */
  ret = Cutter_Cut();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    return ret;
  }
  FruitPick_DelayMs(FRUIT_PICK_CUT_WAIT_MS);

  ret = Cutter_Open();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    return ret;
  }
  FruitPick_DelayMs(FRUIT_PICK_CUT_OPEN_WAIT_MS);

  return FRUIT_ACTUATOR_OK;
}

int FruitPick_PickOne(const FruitTarget_t *fruit)
{
  ArmPoint_t grip;
  ArmPoint_t approach;
  ArmPoint_t drop;
  ArmMotionStatus_t arm_status;
  int ret;

  /* 参数保护：调用者必须传入树莓派识别到的单个果子目标。 */
  if (fruit == NULL)
  {
    return FRUIT_PICK_ERR_PARAM;
  }

  /*
   * 树莓派已经完成相机坐标到机械臂坐标的映射。
   * fruit->x/y/z 是机械臂绝对坐标，单位 mm，
   * STM32 不再做视觉坐标换算，也不再计算额外预备点。
   */
  grip.x_mm = (float)fruit->x_mm;
  grip.y_mm = (float)fruit->y_mm;
  grip.z_mm = (float)fruit->z_mm;

  /* 抓取点必须在 XYZ 软件行程范围内，避免机械臂撞限位或撞结构。 */
  arm_status = Arm_IsPointReachable(&grip);
  if (arm_status != ARM_MOTION_OK)
  {
    return FRUIT_PICK_ERR_UNREACHABLE;
  }

  /*
   * 根据果子类型选择投放准备点。
   * 大果和小果的 XYZ 准备位置不同，不能共用同一个固定投放点。
   */
  ret = FruitPick_GetDropPoint(fruit->type, &drop);
  if (ret != FRUIT_PICK_OK)
  {
    return ret;
  }

  /* 投放准备点也要先做可达性检查，防止抓到果子后才发现无法投放。 */
  arm_status = Arm_IsPointReachable(&drop);
  if (arm_status != ARM_MOTION_OK)
  {
    return FRUIT_PICK_ERR_UNREACHABLE;
  }

  /*
   * 进入抓取流程后使用抓取专用速度/加速度。
   * 当前普通运动和抓取运动都配置为高速，但这里保留单独设置，
   * 方便后续只调整抓取阶段，不影响其它 XYZ 动作。
   */
  arm_status = Arm_SetMotionParams(FRUIT_PICK_ARM_VEL_RPM, FRUIT_PICK_ARM_ACC);
  if (arm_status != ARM_MOTION_OK)
  {
    return FRUIT_PICK_ERR_ACTUATOR;
  }

  /*
   * 抓取前先打开夹爪，避免移动到苹果中心时夹爪处于闭合状态撞到果子。
   * 只要进入高速抓取流程后出错，都要恢复默认运动参数再返回。
   */
  ret = Gripper_Open();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_ACTUATOR;
  }

  /*
   * 抓取前确保剪刀张开。
   * 机械结构已保证夹爪中心对准果心时，剪刀同时对准挂果绳。
   */
  ret = Cutter_Open();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_ACTUATOR;
  }

  /*
   * 分两步移动到树莓派给出的苹果中心抓取点，避免夹爪斜着冲出去撞挂果绳：
   *   第一步：只把 X/Z 移动到抓取点，Y 轴保持当前位置不伸出，
   *           让夹爪先在挂果绳正对面把横向 X 和高度 Z 对齐好。
   *   第二步：等 X/Z 停稳后，再单独把 Y 轴平行地面伸到抓取点，
   *           使挂果绳顺利进入夹爪有效范围。
   * 本流程不再使用 pre point，不让 XYZ 带相机扫描，也不单独计算剪切点。
   */
  arm_status = Arm_GetCurrentPosition(&approach);
  if (arm_status != ARM_MOTION_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_GRIPMOVE;
  }
  approach.x_mm = grip.x_mm;
  approach.z_mm = grip.z_mm;
  /*
   * 第一步的 Y 目标保持当前 Y 不伸出；但 Arm_MoveToPoint() 要求目标 Y 不小于
   * 40mm 保护线，否则会判为不可达，所以这里把 Y 夹到至少 40mm。
   * 正常抓取前 Y 已经在 40mm 安全线，clamp 不会改变行为。
   */
  if (approach.y_mm < ARM_MOTION_Y_PRE_EXTEND_MM)
  {
    approach.y_mm = ARM_MOTION_Y_PRE_EXTEND_MM;
  }

  /* 第一步：X/Z 先到位，Y 不伸出。 */
  arm_status = FruitPick_MoveToPointAbs(&approach);
  if (arm_status != ARM_MOTION_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_GRIPMOVE;
  }
  /* X/Z 停稳等待，确保对齐挂果绳后再伸 Y。 */
  FruitPick_DelayMs(FRUIT_PICK_XZ_ARRIVE_WAIT_MS);

  /* 第二步：X/Z 已对齐，单独把 Y 轴伸到抓取点。 */
  arm_status = FruitPick_MoveYToAbs(grip.y_mm);
  if (arm_status != ARM_MOTION_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_GRIPMOVE;
  }
  /*
   * 必须等 XYZ 彻底到达苹果中心对应的抓取点后再夹爪。
   * Arm_MoveToPoint() 内部已经按距离和速度估算等待时间，
   * 这里再额外等待，防止 Y 轴未完全停稳就夹爪导致夹空。
   * 后续最好接入 Emm_V5 真实到位反馈，替代固定等待。
   */
  FruitPick_DelayMs(FRUIT_PICK_GRIP_ARRIVE_WAIT_MS);

  /* XYZ 停稳后再闭合夹爪，确保苹果被夹住后再进行剪切。 */
  ret = Gripper_Close();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_ACTUATOR;
  }
  FruitPick_DelayMs(FRUIT_PICK_GRIP_WAIT_MS);

  /*
   * 剪刀执行一次剪切并最终保持张开。
   * 如果实车发现挂果绳剪不断，可优先增大剪切等待时间，
   * 或再恢复为二次剪切流程。
   */
  ret = FruitPick_CutOnce();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_ACTUATOR;
  }

  /*
   * 剪刀最终张开后，不抬 Z，直接 Y 轴后撤到 40mm 安全线。
   * 这是当前机械结构已确认的安全撤出方式。
   */
  arm_status = FruitPick_MoveYToAbs(ARM_MOTION_Y_PRE_EXTEND_MM);
  if (arm_status != ARM_MOTION_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_RETRACT_Y;
  }
  FruitPick_DelayMs(FRUIT_PICK_RETRACT_Y_SETTLE_WAIT_MS);

  /*
   * 后撤完成后，再移动到对应果仓的投放准备点。
   * 大果点和小果点在 FruitPick_GetDropPoint() 中已经按类型选好。
   */
  arm_status = FruitPick_MoveToPointAbs(&drop);
  if (arm_status != ARM_MOTION_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_DROP_MOVE;
  }
  FruitPick_DelayMs(FRUIT_PICK_DROP_ARRIVE_WAIT_MS);

  /*
   * 分类舵机转向对应果仓并释放夹爪。
   * 具体舵机角度由 fruit_actuator 模块封装，抓取流程不直接写角度。
   */
  ret = Sorter_DropByType(fruit->type);
  if (ret != FRUIT_ACTUATOR_OK)
  {
    Arm_RestoreDefaultMotionParams();
    return FRUIT_PICK_ERR_DROP;
  }

  /*
   * 投放完成后回到安全回收点。
   * 当前忽略回收返回值：果子已经投放完成，主流程按本次抓取成功返回；
   * 若后续需要严格处理回收失败，可在这里增加错误码。
   */
  (void)Arm_RetractSafe();
  Arm_RestoreDefaultMotionParams();

  return FRUIT_PICK_OK;
}
