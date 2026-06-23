#include "fruit_actuator.h"
#include "servo.h"
#include "cmsis_os.h"

/*
 * 已确认的末端执行器舵机编号和角度。
 *
 * 舵机编号：
 *   夹爪       = SERVO_2
 *   剪刀       = SERVO_1
 *   分类舵机   = SERVO_3
 *   原大果篮/当前小果暂存区 = SERVO_4
 *   原小果篮/当前大果暂存区 = SERVO_5
 *
 * 舵机角度：
 *   夹爪打开   = 255 deg
 *   夹爪闭合   = 100 deg
 *   剪刀打开   = 270 deg
 *   剪刀剪断   = 60 deg
 *   分类居中   = 135 deg
 *   当前大果暂存区方向 = 0 deg（原小果方向）
 *   当前小果暂存区方向 = 270 deg（原大果方向）
 *   原大果篮/当前小果暂存区装载 = 135 deg
 *   原大果篮/当前小果暂存区倒出 = 65 deg
 *   原小果篮/当前大果暂存区装载 = 125 deg
 *   原小果篮/当前大果暂存区倒出 = 170 deg
 */
/* 夹爪舵机：负责夹住/松开苹果。 */
#define FRUIT_ACTUATOR_GRIPPER_SERVO        SERVO_2
/* 剪刀舵机：负责剪断挂苹果的绳子。 */
#define FRUIT_ACTUATOR_CUTTER_SERVO         SERVO_1
/* 分类舵机：负责把夹爪释放方向摆到当前大果或小果暂存区。 */
#define FRUIT_ACTUATOR_SORTER_SERVO         SERVO_3
/* 原大果篮舵机：当前作为小果暂存区使用。 */
#define FRUIT_ACTUATOR_BIG_BASKET_SERVO     SERVO_4
/* 原小果篮舵机：当前作为大果暂存区使用。 */
#define FRUIT_ACTUATOR_SMALL_BASKET_SERVO   SERVO_5

/* 夹爪打开角度：用于抓取前张开，也用于投放时松开苹果。 */
#define FRUIT_ACTUATOR_GRIPPER_OPEN_DEG     255.0f
/* 夹爪闭合角度：用于夹紧苹果。 */
#define FRUIT_ACTUATOR_GRIPPER_CLOSE_DEG    100.0f
/* 剪刀打开角度：抓取前和剪切后都保持在此位置。 */
#define FRUIT_ACTUATOR_CUTTER_OPEN_DEG      270.0f
/* 剪刀剪断角度：执行剪切动作时转到此位置。 */
#define FRUIT_ACTUATOR_CUTTER_CUT_DEG       60.0f
/* 分类舵机居中角度：默认安全位置，避免偏向任意果仓。 */
#define FRUIT_ACTUATOR_SORTER_CENTER_DEG    135.0f
/* 分类舵机当前大果暂存区方向：已换到原小果方向。 */
#define FRUIT_ACTUATOR_SORTER_BIG_DEG       0.0f
/* 分类舵机当前小果暂存区方向：已换到原大果方向。 */
#define FRUIT_ACTUATOR_SORTER_SMALL_DEG     270.0f
/* 原大果篮装载角度：当前用于小果暂存区正常姿态。 */
#define FRUIT_ACTUATOR_BIG_BASKET_LOAD_DEG  135.0f
/* 原大果篮倒出角度：当前用于小果暂存区倒出姿态。 */
#define FRUIT_ACTUATOR_BIG_BASKET_DUMP_DEG  65.0f
/* 原小果篮装载角度：当前用于大果暂存区正常姿态。 */
#define FRUIT_ACTUATOR_SMALL_BASKET_LOAD_DEG 125.0f
/* 原小果篮倒出角度：当前用于大果暂存区倒出姿态。 */
#define FRUIT_ACTUATOR_SMALL_BASKET_DUMP_DEG 170.0f

/* 每次 Servo_SetAngle() 后的基础等待时间，给舵机实际转动留时间。 */
#define FRUIT_ACTUATOR_SERVO_SETTLE_MS      300u
/* 果篮转到倒出角度后保持多久，太短可能倒不干净。 */
#define FRUIT_ACTUATOR_BASKET_DUMP_HOLD_MS  1000u
/* 两次倒果动作之间的间隔，先回装载姿态再进行第二次倒出。 */
#define FRUIT_ACTUATOR_BASKET_DUMP_GAP_MS   500u

/*
 * 分类舵机转到当前大果/小果暂存区方向后，等待多久再打开夹爪，单位 ms。
 *
 * 这个参数专门解决“分类舵机还没转到位，夹爪就已经松开”的问题。
 * 如果果子还没对准果仓就掉落，增大这里；如果确认舵机到位很快，可以减小。
 */
#define FRUIT_ACTUATOR_SORTER_BEFORE_RELEASE_WAIT_MS  1000u

/*
 * 夹爪打开、果子掉入果仓后，等待多久再让分类舵机回中，单位 ms。
 * 如果果子释放后容易被回中的分类机构碰到，可以增大这里。
 */
#define FRUIT_ACTUATOR_SORTER_AFTER_RELEASE_WAIT_MS   500u

static void FruitActuator_DelayMs(uint32_t delay_ms)
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

static int FruitActuator_SetAngle(servo_id_e servo_id, float angle_deg)
{
  if (Servo_SetAngle(servo_id, angle_deg) != 0)
  {
    return FRUIT_ACTUATOR_ERR_SERVO;
  }

  FruitActuator_DelayMs(FRUIT_ACTUATOR_SERVO_SETTLE_MS);
  return FRUIT_ACTUATOR_OK;
}

static int FruitBasket_DumpRepeat(servo_id_e servo_id,
                                  float load_angle_deg,
                                  float dump_angle_deg,
                                  uint8_t repeat_count)
{
  int ret;
  uint8_t i;

  for (i = 0u; i < repeat_count; i++)
  {
    ret = FruitActuator_SetAngle(servo_id, dump_angle_deg);
    if (ret != FRUIT_ACTUATOR_OK)
    {
      return ret;
    }
    FruitActuator_DelayMs(FRUIT_ACTUATOR_BASKET_DUMP_HOLD_MS);

    ret = FruitActuator_SetAngle(servo_id, load_angle_deg);
    if (ret != FRUIT_ACTUATOR_OK)
    {
      return ret;
    }

    if ((i + 1u) < repeat_count)
    {
      FruitActuator_DelayMs(FRUIT_ACTUATOR_BASKET_DUMP_GAP_MS);
    }
  }

  return FRUIT_ACTUATOR_OK;
}

int Gripper_Open(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_GRIPPER_SERVO,
                                FRUIT_ACTUATOR_GRIPPER_OPEN_DEG);
}

int Gripper_Close(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_GRIPPER_SERVO,
                                FRUIT_ACTUATOR_GRIPPER_CLOSE_DEG);
}

int Cutter_Open(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_CUTTER_SERVO,
                                FRUIT_ACTUATOR_CUTTER_OPEN_DEG);
}

int Cutter_Cut(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_CUTTER_SERVO,
                                FRUIT_ACTUATOR_CUTTER_CUT_DEG);
}

int Sorter_ToCenter(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_SORTER_SERVO,
                                FRUIT_ACTUATOR_SORTER_CENTER_DEG);
}

int Sorter_ToBigSide(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_SORTER_SERVO,
                                FRUIT_ACTUATOR_SORTER_BIG_DEG);
}

int Sorter_ToSmallSide(void)
{
  return FruitActuator_SetAngle(FRUIT_ACTUATOR_SORTER_SERVO,
                                FRUIT_ACTUATOR_SORTER_SMALL_DEG);
}

int Sorter_DropByType(FruitType_t type)
{
  int ret;

  if (type == FRUIT_BIG)
  {
    ret = Sorter_ToBigSide();
  }
  else if (type == FRUIT_SMALL)
  {
    ret = Sorter_ToSmallSide();
  }
  else
  {
    return FRUIT_ACTUATOR_ERR_PARAM;
  }

  if (ret != FRUIT_ACTUATOR_OK)
  {
    return ret;
  }

  /*
   * 分类舵机先转到对应果仓方向，并额外等待到位后，
   * 再打开夹爪释放果子。
   */
  FruitActuator_DelayMs(FRUIT_ACTUATOR_SORTER_BEFORE_RELEASE_WAIT_MS);

  ret = Gripper_Open();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    return ret;
  }

  FruitActuator_DelayMs(FRUIT_ACTUATOR_SORTER_AFTER_RELEASE_WAIT_MS);
  return Sorter_ToCenter();
}

/**
  * @brief 大果暂存篮倒出苹果。
  * @note  当前大果暂存区已换到原小果篮 SERVO_5。
  *        SERVO_5 先从装载角度 125 度转到倒出角度 170 度，
  *        等待果子滑出后再回到 125 度；整套动作会执行两次，
  *        用于减少因为速度太快或果子卡住导致倒不干净的情况。
  * @retval FRUIT_ACTUATOR_OK 倒出动作完成，且最终回到装载状态。
  * @retval FRUIT_ACTUATOR_ERR_SERVO 舵机角度设置失败。
  */
int FruitBasket_DumpBig(void)
{
  return FruitBasket_DumpRepeat(FRUIT_ACTUATOR_SMALL_BASKET_SERVO,
                                FRUIT_ACTUATOR_SMALL_BASKET_LOAD_DEG,
                                FRUIT_ACTUATOR_SMALL_BASKET_DUMP_DEG,
                                2u);
}

/**
  * @brief 小果暂存篮倒出苹果。
  * @note  当前小果暂存区已换到原大果篮 SERVO_4。
  *        SERVO_4 先从装载角度 135 度转到倒出角度 65 度，
  *        等待果子滑出后再回到 135 度；整套动作会执行三次，
  *        用于减少因为速度太快或果子卡住导致倒不干净的情况。
  * @retval FRUIT_ACTUATOR_OK 倒出动作完成，且最终回到装载状态。
  * @retval FRUIT_ACTUATOR_ERR_SERVO 舵机角度设置失败。
  */
int FruitBasket_DumpSmall(void)
{
  return FruitBasket_DumpRepeat(FRUIT_ACTUATOR_BIG_BASKET_SERVO,
                                FRUIT_ACTUATOR_BIG_BASKET_LOAD_DEG,
                                FRUIT_ACTUATOR_BIG_BASKET_DUMP_DEG,
                                3u);
}

/**
  * @brief 依次倒出大果暂存篮和小果暂存篮。
  * @note  先执行 FruitBasket_DumpBig()，成功后再执行 FruitBasket_DumpSmall()。
  *        如果大果篮倒出失败，会立即返回错误，不继续倒小果篮。
  *        当前大果暂存区执行两次倒出动作，当前小果暂存区执行三次倒出动作，
  *        并最终回到装载状态。
  * @retval FRUIT_ACTUATOR_OK 两个暂存篮都倒出完成。
  * @retval FRUIT_ACTUATOR_ERR_SERVO 任意一个果篮舵机角度设置失败。
  */
int FruitBasket_DumpAll(void)
{
  int ret;

  ret = FruitBasket_DumpBig();
  if (ret != FRUIT_ACTUATOR_OK)
  {
    return ret;
  }

  return FruitBasket_DumpSmall();
}
