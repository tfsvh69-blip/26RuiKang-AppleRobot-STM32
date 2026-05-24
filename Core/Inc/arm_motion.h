/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    arm_motion.h
  * @brief   基于 Emm_V5 闭环步进电机的 XYZ 轴运动封装
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ARM_MOTION_H__
#define __ARM_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* XYZ 轴电机 ID 映射 */
#define ARM_MOTION_MOTOR_Z                  5u
#define ARM_MOTION_MOTOR_Y                  6u
#define ARM_MOTION_MOTOR_X                  7u

/* Emm_V5_Pos_Control() 正方向参数映射 */
#define ARM_MOTION_DIR_Z_POS                0u
#define ARM_MOTION_DIR_Y_POS                1u
#define ARM_MOTION_DIR_X_POS                1u

/* 机械换算关系 */
#define ARM_MOTION_XY_PULSES_PER_REV        3200u
#define ARM_MOTION_XY_MM_PER_REV            40.0f
#define ARM_MOTION_Z_PULSES_PER_REV         3200u
#define ARM_MOTION_Z_MM_PER_REV             10.0f

#define ARM_MOTION_X_PULSES_PER_MM          (ARM_MOTION_XY_PULSES_PER_REV / ARM_MOTION_XY_MM_PER_REV)
#define ARM_MOTION_Y_PULSES_PER_MM          (ARM_MOTION_XY_PULSES_PER_REV / ARM_MOTION_XY_MM_PER_REV)
#define ARM_MOTION_Z_PULSES_PER_MM          (ARM_MOTION_Z_PULSES_PER_REV / ARM_MOTION_Z_MM_PER_REV)

/*
 * 默认 Emm_V5 运动参数。
 * Arm_Init_AllParallel() 回零流程在 Arm.c 内部仍使用独立的 200rpm / acc=20，
 * 这里的默认值用于回零完成后的普通 XYZ 绝对位置运动、后撤、投放和安全回收。
 */
#define ARM_MOTION_DEFAULT_VEL_RPM          800u
#define ARM_MOTION_DEFAULT_ACC              80u

/* 已确认的 XYZ 机械行程范围，单位 mm。 */
#define ARM_MOTION_X_MIN_MM                 0.0f
#define ARM_MOTION_X_MAX_MM                 220.0f
#define ARM_MOTION_Y_MIN_MM                 0.0f
#define ARM_MOTION_Y_MAX_MM                 480.0f
#define ARM_MOTION_Z_MIN_MM                 0.0f
#define ARM_MOTION_Z_MAX_MM                 400.0f

/*
 * 已确认的安全回收点，单位 mm。
 * 注意：这里的 SAFE 是 Arm_RetractSafe() 使用的回收坐标，
 * 不是 XYZ 最大行程。最大行程见 ARM_MOTION_X/Y/Z_MAX_MM。
 */
#define ARM_MOTION_SAFE_X_MM                0.0f
#define ARM_MOTION_SAFE_Y_MM                40.0f
#define ARM_MOTION_SAFE_Z_MM                0.0f

#define ARM_MOTION_Y_EXTEND_SAFE_MM         400.0f
#define ARM_MOTION_Z_SAFE_LIFT_MM           40.0f

/*
 * 运动保护距离：
 * 上电回零完成后，先把 Y 轴移动到 40mm。
 * 后续调用 Arm_MoveToPoint() 时，目标 Y 轴不允许小于 40mm；
 * 如果当前 Y 轴还没有伸出到 40mm，也会先移动到 40mm，再移动 X/Z。
 */
#define ARM_MOTION_Y_PRE_EXTEND_MM          40.0f

/*
 * 当前工程还没有封装 Emm_V5 运动完成反馈。
 * 这里的等待时间只是用于一轴一轴运动时的估算间隔。
 * 等待估算会在理论匀速时间基础上乘安全倍率，再加固定余量，
 * 用来补偿加减速、驱动响应、机械惯性和串口发送间隔。
 *
 * 速度调试说明：
 *   如果电机已经到位但下一轴迟迟不动，说明这里等待偏保守，可以小幅降低；
 *   如果上一轴还没到位下一轴就开始动，必须立刻增大这里，直到重新接入真实到位反馈。
 */
#define ARM_MOTION_MIN_AXIS_WAIT_MS         50u
#define ARM_MOTION_EXTRA_AXIS_WAIT_MS       120u
#define ARM_MOTION_WAIT_SCALE_PERCENT       120u

typedef enum
{
  ARM_MOTION_OK = 0,                 /* 操作成功 */
  ARM_MOTION_ERR_PARAM = -1,         /* 参数错误，例如空指针 */
  ARM_MOTION_ERR_UNREACHABLE = -2,   /* 目标点超出可达范围 */
  ARM_MOTION_ERR_HOME = -3,          /* 回零/寻限位失败 */
  ARM_MOTION_ERR_AXIS = -4,          /* 轴编号错误 */
  ARM_MOTION_ERR_NO_FEEDBACK = -5    /* 预留：底层没有运动完成反馈 */
} ArmMotionStatus_t;

typedef enum
{
  ARM_AXIS_X = 0,                    /* X 轴，电机 ID 7 */
  ARM_AXIS_Y = 1,                    /* Y 轴，电机 ID 6 */
  ARM_AXIS_Z = 2                     /* Z 轴，电机 ID 5，丝杆高度轴 */
} ArmAxis_t;

typedef struct
{
  float x_mm;                        /* X 轴坐标，单位 mm */
  float y_mm;                        /* Y 轴坐标，单位 mm */
  float z_mm;                        /* Z 轴坐标，单位 mm */
} ArmPoint_t;

/**
  * @brief  将当前 XYZ 位置设置为软件零点，并同步清零 5/6/7 号电机当前位置。
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_SetCurrentPositionZero(void);

/**
  * @brief  获取模块缓存的当前 XYZ 软件坐标。
  * @param  position 输出坐标，单位 mm
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_GetCurrentPosition(ArmPoint_t *position);

/**
  * @brief  设置后续 XYZ 运动使用的速度和加速度。
  * @param  vel_rpm Emm_V5 速度参数，单位 rpm
  * @param  acc Emm_V5 加速度参数
  * @note   只修改 arm_motion 模块后续发给 Emm_V5 的参数，不改变当前位置。
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_SetMotionParams(uint16_t vel_rpm, uint8_t acc);

/**
  * @brief  恢复默认 XYZ 运动速度和加速度。
  */
void Arm_RestoreDefaultMotionParams(void);

/**
  * @brief  判断目标点是否在软件限制范围内。
  * @param  point 目标坐标，单位 mm
  * @retval ARM_MOTION_OK 表示可达，否则返回错误码
  */
ArmMotionStatus_t Arm_IsPointReachable(const ArmPoint_t *point);

/**
  * @brief  指定单轴相对移动。
  * @param  axis 轴编号
  * @param  distance_mm 相对位移，单位 mm，正负号表示方向
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Axis_MoveRelMm(ArmAxis_t axis, float distance_mm);

/**
  * @brief  移动到 XYZ 绝对坐标点。
  * @note   除上电回零外，目标 Y 轴必须大于等于 40mm；
  *         若当前 Y 轴小于 40mm，会先移动 Y 到 40mm，再移动其它轴。
  * @param  x_mm X 轴目标绝对坐标，单位 mm
  * @param  y_mm Y 轴目标绝对坐标，单位 mm
  * @param  z_mm Z 轴目标绝对坐标，单位 mm
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_MoveToPoint(float x_mm, float y_mm, float z_mm);

/**
  * @brief  Y 轴相对移动快捷接口。
  * @param  distance_mm Y 轴相对位移，单位 mm，正方向为向果树伸出
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_MoveYRel(float distance_mm);

/**
  * @brief  安全回收接口。
  * @note   已确认回收点为 X=0mm, Y=40mm, Z=0mm。
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_RetractSafe(void);

/**
  * @brief  XYZ 轴回零。
  * @note   内部调用 Arm_Init_AllParallel() 寻限位，然后将当前位置设为软件零点。
  * @retval ArmMotionStatus_t 错误码
  */
ArmMotionStatus_t Arm_HomeXYZ(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_MOTION_H__ */
