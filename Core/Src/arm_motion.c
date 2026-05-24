/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    arm_motion.c
  * @brief   基于 Emm_V5 闭环步进电机的 XYZ 轴运动封装
  ******************************************************************************
  */
/* USER CODE END Header */

#include "arm_motion.h"
#include "Arm.h"
#include "Emm_V5.h"
#include "cmsis_os.h"

static ArmPoint_t g_arm_current_position = {0.0f, 0.0f, 0.0f};
static uint16_t g_arm_motion_vel_rpm = ARM_MOTION_DEFAULT_VEL_RPM;
static uint8_t g_arm_motion_acc = ARM_MOTION_DEFAULT_ACC;

/* 目标和当前位置差值小于该值时，认为该轴不需要运动，避免空命令和空等待。 */
#define ARM_MOTION_AXIS_NO_MOVE_EPSILON_MM  0.01f

/* RTOS 感知延时：调度器运行后用 osDelay，初始化阶段用 HAL_Delay。 */
static void ArmMotion_DelayMs(uint32_t delay_ms)
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

/* 取 float 绝对值，避免额外依赖 math 库。 */
static float ArmMotion_AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

/* 根据正方向参数得到反方向参数，Emm_V5 方向只有 0/1 两种。 */
static uint8_t ArmMotion_GetNegativeDir(uint8_t positive_dir)
{
  return (positive_dir == 0u) ? 1u : 0u;
}

/* float 四舍五入到 uint32_t，用于 mm -> 脉冲后的整数化。 */
static uint32_t ArmMotion_RoundToU32(float value)
{
  if (value <= 0.0f)
  {
    return 0u;
  }

  return (uint32_t)(value + 0.5f);
}

/* 将某轴的毫米位移绝对值换算为脉冲数。 */
static uint32_t ArmMotion_MmToPulses(ArmAxis_t axis, float mm_abs)
{
  switch (axis)
  {
    case ARM_AXIS_X:
      return ArmMotion_RoundToU32(mm_abs * ARM_MOTION_X_PULSES_PER_MM);

    case ARM_AXIS_Y:
      return ArmMotion_RoundToU32(mm_abs * ARM_MOTION_Y_PULSES_PER_MM);

    case ARM_AXIS_Z:
      return ArmMotion_RoundToU32(mm_abs * ARM_MOTION_Z_PULSES_PER_MM);

    default:
      return 0u;
  }
}

/*
 * 按速度和位移估算单轴运动等待时间，确保逐轴动作之间留出间隔。
 * 计算方法：
 *   1. mm -> 脉冲 -> 圈数；
 *   2. 根据当前 rpm 计算理论匀速时间；
 *   3. 乘安全倍率，再加固定余量，补偿加减速和机械响应。
 */
static uint32_t ArmMotion_EstimateAxisWaitMs(ArmAxis_t axis, float delta_mm)
{
  float pulses;
  float rev;
  float move_ms;
  float pulses_per_rev;

  pulses = (float)ArmMotion_MmToPulses(axis, ArmMotion_AbsFloat(delta_mm));

  if (axis == ARM_AXIS_Z)
  {
    pulses_per_rev = (float)ARM_MOTION_Z_PULSES_PER_REV;
  }
  else
  {
    pulses_per_rev = (float)ARM_MOTION_XY_PULSES_PER_REV;
  }

  rev = pulses / pulses_per_rev;
  move_ms = (rev * 60000.0f) / (float)g_arm_motion_vel_rpm;
  move_ms = (move_ms * (float)ARM_MOTION_WAIT_SCALE_PERCENT) / 100.0f;
  move_ms += (float)ARM_MOTION_EXTRA_AXIS_WAIT_MS;

  if (move_ms < (float)ARM_MOTION_MIN_AXIS_WAIT_MS)
  {
    move_ms = (float)ARM_MOTION_MIN_AXIS_WAIT_MS;
  }

  return ArmMotion_RoundToU32(move_ms);
}

/* 查询轴对应的电机 ID 和 Emm_V5 正方向参数。 */
static ArmMotionStatus_t ArmMotion_GetAxisConfig(ArmAxis_t axis,
                                                 uint8_t *motor_id,
                                                 uint8_t *positive_dir)
{
  if ((motor_id == NULL) || (positive_dir == NULL))
  {
    return ARM_MOTION_ERR_PARAM;
  }

  switch (axis)
  {
    case ARM_AXIS_X:
      *motor_id = ARM_MOTION_MOTOR_X;
      *positive_dir = ARM_MOTION_DIR_X_POS;
      return ARM_MOTION_OK;

    case ARM_AXIS_Y:
      *motor_id = ARM_MOTION_MOTOR_Y;
      *positive_dir = ARM_MOTION_DIR_Y_POS;
      return ARM_MOTION_OK;

    case ARM_AXIS_Z:
      *motor_id = ARM_MOTION_MOTOR_Z;
      *positive_dir = ARM_MOTION_DIR_Z_POS;
      return ARM_MOTION_OK;

    default:
      return ARM_MOTION_ERR_AXIS;
  }
}

/* 从坐标结构中读取指定轴坐标。 */
static float ArmMotion_GetAxisPosition(const ArmPoint_t *point, ArmAxis_t axis)
{
  switch (axis)
  {
    case ARM_AXIS_X:
      return point->x_mm;

    case ARM_AXIS_Y:
      return point->y_mm;

    case ARM_AXIS_Z:
      return point->z_mm;

    default:
      return 0.0f;
  }
}

/* 写入坐标结构中的指定轴坐标。 */
static void ArmMotion_SetAxisPosition(ArmPoint_t *point, ArmAxis_t axis, float value_mm)
{
  switch (axis)
  {
    case ARM_AXIS_X:
      point->x_mm = value_mm;
      break;

    case ARM_AXIS_Y:
      point->y_mm = value_mm;
      break;

    case ARM_AXIS_Z:
      point->z_mm = value_mm;
      break;

    default:
      break;
  }
}

/* 单轴绝对位置运动：换算脉冲后调用 Emm_V5_Pos_Control()，raF=true。 */
static ArmMotionStatus_t ArmMotion_MoveAxisAbs(ArmAxis_t axis, float target_mm)
{
  uint8_t motor_id;
  uint8_t positive_dir;
  uint8_t dir;
  uint32_t pulses;
  float current_mm;
  float delta_mm;
  ArmMotionStatus_t status;

  status = ArmMotion_GetAxisConfig(axis, &motor_id, &positive_dir);
  if (status != ARM_MOTION_OK)
  {
    return status;
  }

  current_mm = ArmMotion_GetAxisPosition(&g_arm_current_position, axis);
  delta_mm = target_mm - current_mm;

  if (ArmMotion_AbsFloat(delta_mm) <= ARM_MOTION_AXIS_NO_MOVE_EPSILON_MM)
  {
    return ARM_MOTION_OK;
  }

  dir = (target_mm >= 0.0f) ? positive_dir : ArmMotion_GetNegativeDir(positive_dir);
  pulses = ArmMotion_MmToPulses(axis, ArmMotion_AbsFloat(target_mm));

  Emm_V5_Pos_Control(motor_id,
                     dir,
                     g_arm_motion_vel_rpm,
                     g_arm_motion_acc,
                     pulses,
                     true,
                     false);

  /*
   * TODO: 等底层增加 Emm_V5 运动完成/状态读取接口后，
   *       用真实反馈替代这里的估算等待。
   */
  ArmMotion_DelayMs(ArmMotion_EstimateAxisWaitMs(axis, delta_mm));
  ArmMotion_SetAxisPosition(&g_arm_current_position, axis, target_mm);

  return ARM_MOTION_OK;
}

ArmMotionStatus_t Arm_SetCurrentPositionZero(void)
{
  Emm_V5_Reset_CurPos_To_Zero(ARM_MOTION_MOTOR_X);
  Emm_V5_Reset_CurPos_To_Zero(ARM_MOTION_MOTOR_Y);
  Emm_V5_Reset_CurPos_To_Zero(ARM_MOTION_MOTOR_Z);

  g_arm_current_position.x_mm = 0.0f;
  g_arm_current_position.y_mm = 0.0f;
  g_arm_current_position.z_mm = 0.0f;

  return ARM_MOTION_OK;
}

ArmMotionStatus_t Arm_GetCurrentPosition(ArmPoint_t *position)
{
  if (position == NULL)
  {
    return ARM_MOTION_ERR_PARAM;
  }

  *position = g_arm_current_position;
  return ARM_MOTION_OK;
}

ArmMotionStatus_t Arm_SetMotionParams(uint16_t vel_rpm, uint8_t acc)
{
  if ((vel_rpm == 0u) || (acc == 0u))
  {
    return ARM_MOTION_ERR_PARAM;
  }

  g_arm_motion_vel_rpm = vel_rpm;
  g_arm_motion_acc = acc;
  return ARM_MOTION_OK;
}

void Arm_RestoreDefaultMotionParams(void)
{
  g_arm_motion_vel_rpm = ARM_MOTION_DEFAULT_VEL_RPM;
  g_arm_motion_acc = ARM_MOTION_DEFAULT_ACC;
}

ArmMotionStatus_t Arm_IsPointReachable(const ArmPoint_t *point)
{
  if (point == NULL)
  {
    return ARM_MOTION_ERR_PARAM;
  }

  if ((point->x_mm < ARM_MOTION_X_MIN_MM) || (point->x_mm > ARM_MOTION_X_MAX_MM) ||
      (point->y_mm < ARM_MOTION_Y_MIN_MM) || (point->y_mm > ARM_MOTION_Y_MAX_MM) ||
      (point->z_mm < ARM_MOTION_Z_MIN_MM) || (point->z_mm > ARM_MOTION_Z_MAX_MM))
  {
    return ARM_MOTION_ERR_UNREACHABLE;
  }

  return ARM_MOTION_OK;
}

ArmMotionStatus_t Axis_MoveRelMm(ArmAxis_t axis, float distance_mm)
{
  ArmPoint_t target;
  ArmMotionStatus_t status;

  target = g_arm_current_position;
  ArmMotion_SetAxisPosition(&target,
                            axis,
                            ArmMotion_GetAxisPosition(&target, axis) + distance_mm);

  status = Arm_IsPointReachable(&target);
  if (status != ARM_MOTION_OK)
  {
    return status;
  }

  return ArmMotion_MoveAxisAbs(axis, ArmMotion_GetAxisPosition(&target, axis));
}

ArmMotionStatus_t Arm_MoveToPoint(float x_mm, float y_mm, float z_mm)
{
  ArmPoint_t point;
  ArmMotionStatus_t status;
  float safe_z_mm;

  point.x_mm = x_mm;
  point.y_mm = y_mm;
  point.z_mm = z_mm;

  status = Arm_IsPointReachable(&point);
  if (status != ARM_MOTION_OK)
  {
    return status;
  }

  /*
   * 除 Arm_HomeXYZ() 上电回零过程外，后续普通 XYZ 动作不允许
   * 目标 Y 轴小于 40mm，避免机械臂再次收回到零点附近。
   */
  if (point.y_mm < ARM_MOTION_Y_PRE_EXTEND_MM)
  {
    return ARM_MOTION_ERR_UNREACHABLE;
  }

  /*
   * 运动顺序保护：
   * 如果当前 Y 轴还在 40mm 以内，先把 Y 轴伸出到 40mm，
   * 再开始移动 Z/X 等其它轴。这样可以避免机械臂贴近零点时，
   * 其它轴先动作导致末端结构碰到车体或机构。
   */
  if (g_arm_current_position.y_mm < ARM_MOTION_Y_PRE_EXTEND_MM)
  {
    status = ArmMotion_MoveAxisAbs(ARM_AXIS_Y, ARM_MOTION_Y_PRE_EXTEND_MM);
    if (status != ARM_MOTION_OK)
    {
      return status;
    }
  }

  if (point.y_mm > ARM_MOTION_Y_EXTEND_SAFE_MM)
  {
    safe_z_mm = g_arm_current_position.z_mm + ARM_MOTION_Z_SAFE_LIFT_MM;
    if (safe_z_mm > ARM_MOTION_Z_MAX_MM)
    {
      return ARM_MOTION_ERR_UNREACHABLE;
    }

    status = ArmMotion_MoveAxisAbs(ARM_AXIS_Z, safe_z_mm);
    if (status != ARM_MOTION_OK)
    {
      return status;
    }
  }
  else
  {
    status = ArmMotion_MoveAxisAbs(ARM_AXIS_Z, point.z_mm);
    if (status != ARM_MOTION_OK)
    {
      return status;
    }
  }

  status = ArmMotion_MoveAxisAbs(ARM_AXIS_X, point.x_mm);
  if (status != ARM_MOTION_OK)
  {
    return status;
  }

  status = ArmMotion_MoveAxisAbs(ARM_AXIS_Y, point.y_mm);
  if (status != ARM_MOTION_OK)
  {
    return status;
  }

  if (point.y_mm > ARM_MOTION_Y_EXTEND_SAFE_MM)
  {
    status = ArmMotion_MoveAxisAbs(ARM_AXIS_Z, point.z_mm);
    if (status != ARM_MOTION_OK)
    {
      return status;
    }
  }

  return ARM_MOTION_OK;
}

ArmMotionStatus_t Arm_MoveYRel(float distance_mm)
{
  return Axis_MoveRelMm(ARM_AXIS_Y, distance_mm);
}

ArmMotionStatus_t Arm_RetractSafe(void)
{
  return Arm_MoveToPoint(ARM_MOTION_SAFE_X_MM,
                         ARM_MOTION_SAFE_Y_MM,
                         ARM_MOTION_SAFE_Z_MM);
}

ArmMotionStatus_t Arm_HomeXYZ(void)
{
  if (Arm_Init_AllParallel() != 0)
  {
    return ARM_MOTION_ERR_HOME;
  }

  return Arm_SetCurrentPositionZero();
}
