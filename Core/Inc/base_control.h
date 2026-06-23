#ifndef __BASE_CONTROL_H__
#define __BASE_CONTROL_H__

#include "main.h"
#include "pi_protocol.h"

#define BASE_ROTATE_DIR_CW       (0u)
#define BASE_ROTATE_DIR_CCW      (1u)
#define BASE_ROTATE_DIR_SHORTEST (2u)
#define BASE_FOLLOW_SIDE_LEFT  (0u)
#define BASE_FOLLOW_SIDE_RIGHT (1u)

/* 前方雷达停车调试变量：用于 ST-Link Watch 观察 dist1 原始状态。 */
extern volatile int16_t g_base_front_lidar_raw_distance_mm;
extern volatile uint8_t g_base_front_lidar_status;
extern volatile uint32_t g_base_front_lidar_timestamp;
extern volatile uint8_t g_base_front_lidar_valid;
extern volatile uint8_t g_base_front_lidar_stop_confirm_count;

/* 相对当前 yaw 旋转，正数=顺时针，负数=逆时针。 */
int Base_RotateToYaw(float target_yaw, int16_t max_speed_rrp, float kp, float kd);
/* 旋转到相对本次 MCU 复位零点的 yaw 角度，输入范围建议 -180 到 +180，内部自动选择最短旋转方向。 */
int Base_RotateToAbsYaw(float target_yaw_abs, int16_t max_speed_rrp, float kp, float kd);
int Base_FollowWall(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float kd);
/* slow_dist_mm: 距离前方停止线多少 mm 开始线性减速；值越大，靠近越柔和。 */
int Base_FollowWallWithSlowdown(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float kd, uint16_t slow_dist_mm);
/* 带I项的贴墙控制；ki从0开始小幅增加，用于消除长期侧距偏差。 */
int Base_FollowWallPidWithSlowdown(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float ki, float kd, uint16_t slow_dist_mm);
/* 按相对本次 MCU 复位零点的 yaw 角度保持航向前进，直到正前方距离小于停止阈值。 */
int Base_ForwardHoldAbsYaw(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd);
int Base_ForwardHoldAbsYawWithSlowdown(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd, uint16_t slow_dist_mm);
int Base_ForwardHoldAbsYawPidWithSlowdown(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float ki, float kd, uint16_t slow_dist_mm);
int Base_ForwardUntilFrontDistance(uint16_t stop_dist_mm, int16_t forward_speed_rrp);
/* 按相对本次 MCU 复位零点的 yaw 角度闭环前进直到前方距离小于阈值，使用 Base_RotateToGoalYaw 风格 PD 控制（带减速区/精调区/最小速度） */
int Base_ForwardUntilFrontDistanceHoldYaw(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd, uint32_t min_run_time_ms);

/* 按相对本次 MCU 复位零点的 yaw 前进到前方距离阈值，前进过程中监听 Pi 的 HIT；命中返回 1，前方距离到达返回 0。 */
int Base_ForwardUntilFrontDistanceHoldYawWatchPi(uint16_t stop_dist_mm,
                                                 int16_t forward_speed_rrp,
                                                 float target_yaw_abs,
                                                 float kp,
                                                 float kd,
                                                 uint8_t tree_id,
                                                 TreeViewId_t view_id,
                                                 uint32_t min_run_time_ms);

/* 按指定距离(cm)闭环前进，同时用 PD 保持相对本次 MCU 复位零点的 yaw 角度；内部仍用估算脉冲积分退出。 */
int Base_ForwardPulseHoldYaw(float distance_cm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd);

/*
 * 底盘位置模式按脉冲前进/后退，不做距离换算，不做陀螺仪闭环。
 * pulse > 0 表示前进，pulse < 0 表示后退；vel_rpm 和 acc 直接下发给 Emm_V5。
 */
int Base_ForwardPulsePosition(int32_t pulse, uint16_t vel_rpm, uint8_t acc);

/*
 * 底盘位置模式按 cm 前进/后退，不做陀螺仪闭环。
 * distance_cm > 0 表示前进，distance_cm < 0 表示后退。
 * 换算基于实测：6400 脉冲约等于 98cm。
 */
int Base_ForwardDistanceCmNoYaw(float distance_cm, int16_t vel_rpm, uint8_t acc);

/* 按指定距离(cm)闭环前进，同时用 PD 保持相对本次 MCU 复位零点的 yaw 角度。 */
int Base_ForwardDistanceCmHoldYaw(float distance_cm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd);

/* 保持相对本次 MCU 复位零点的 yaw 前进指定距离，前进过程中监听 Pi 的 HIT，命中后立即停车。 */
int Base_ForwardDistanceCmHoldYawWatchPi(float distance_cm,
                                         int16_t forward_speed_rrp,
                                         float target_yaw_abs,
                                         float kp,
                                         float kd,
                                         uint8_t tree_id,
                                         TreeViewId_t view_id,
                                         float *traveled_cm_out);

#endif
