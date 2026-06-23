
#include "base_control.h"
#include "Emm_V5.h"
#include "cmsis_os.h"
#include "lidar_manager.h"
#include "usart.h"
#include "pi_protocol.h"

// 容许的角度误差（度），小于该值认为到达目标
#define BASE_ROTATE_YAW_TOLERANCE_DEG  (1.0f)
// 停车复查窗口（度），进入后先停车，等待惯性释放后再判断是否退出
#define BASE_ROTATE_STOP_WINDOW_DEG    (1.8f)
// 控制周期，单位ms
#define BASE_ROTATE_CTRL_PERIOD_MS    (10u)
// 连续稳定时间（ms），超过该时间才判定到位；值越小，旋转到位后退出越快
#define BASE_ROTATE_STABLE_TIME_MS    (60u)
// 进入目标附近后停车等待时间，给底盘惯性和IMU刷新留时间
#define BASE_ROTATE_SETTLE_WAIT_MS    (80u)
// 最小输出速度，防止速度过低无法转动
#define BASE_ROTATE_MIN_SPEED_RRP     (7)
// 精调区最大速度，避免接近目标时左右来回冲过头
#define BASE_ROTATE_FINE_MAX_SPEED_RRP (12)
// 进入减速区的角度阈值（度），越接近目标速度越小
#define BASE_ROTATE_SLOW_ZONE_DEG     (35.0f)
// 精调区间（度），进入后允许小幅反向修正
#define BASE_ROTATE_FINE_ZONE_DEG     (3.0f)
// 角速度变化很小也认为稳定
#define BASE_ROTATE_DERR_TOL          (0.2f)
// 超时时间，ms（设为0表示不超时）
#define BASE_ROTATE_TIMEOUT_MS        (0u)

// 前进+yaw保持控制周期，单位ms
#define BASE_FWDYAW_CTRL_PERIOD_MS    (10u)
// 前进+yaw保持最小修正速度，防止误差太小时完全不修正
#define BASE_FWDYAW_MIN_SPEED_RRP     (3)
// 前进+yaw保持精调区最大修正速度
#define BASE_FWDYAW_FINE_MAX_SPEED_RRP (8)
// 前进+yaw保持减速区角度阈值（度）
#define BASE_FWDYAW_SLOW_ZONE_DEG     (15.0f)
// 前进+yaw保持精调区角度阈值（度）
#define BASE_FWDYAW_FINE_ZONE_DEG     (2.0f)
// 底盘轮子参数：3200脉冲/圈，轮径150mm。
#define BASE_DRIVE_PULSE_PER_REV      (3200.0f)
#define BASE_DRIVE_WHEEL_DIAMETER_MM  (150.0f)
#define BASE_DRIVE_PI                 (3.1415926f)
#define BASE_DRIVE_PULSE_PER_CM       (BASE_DRIVE_PULSE_PER_REV / ((BASE_DRIVE_WHEEL_DIAMETER_MM * BASE_DRIVE_PI) / 10.0f))
// 实测：Base_ForwardPulseHoldYaw(10cm, 90rrp) 实际约前进16cm，按 10/16 修正。
#define BASE_DRIVE_DISTANCE_CALIBRATION (0.3125f)
// 指定距离前进时，离目标还剩这些距离开始降速，减小停车冲击和震动。
#define BASE_DRIVE_STOP_SLOW_ZONE_CM    (10.0f)
#define BASE_DRIVE_STOP_MIN_SPEED_PERCENT (60u)
// 底盘位置模式电机 ID：ID1/2 与 ID3/4 的方向按 Emm_V5_Motor_Control() 现有映射推导。
#define BASE_DRIVE_MOTOR_1              (1u)
#define BASE_DRIVE_MOTOR_2              (2u)
#define BASE_DRIVE_MOTOR_3              (3u)
#define BASE_DRIVE_MOTOR_4              (4u)
#define BASE_DRIVE_FORWARD_DIR_12       (1u)
#define BASE_DRIVE_BACKWARD_DIR_12      (0u)
#define BASE_DRIVE_FORWARD_DIR_34       (0u)
#define BASE_DRIVE_BACKWARD_DIR_34      (1u)
#define BASE_DRIVE_POS_EXTRA_WAIT_MS    (500u)
// 位置模式实测换算：Base_ForwardPulsePosition(3200*2, 80, 40) 实际约走 98cm。
#define BASE_DRIVE_POSITION_PULSE_PER_CM (6400.0f / 98.0f)

// 贴墙前进控制周期，单位ms
#define BASE_FOLLOW_CTRL_PERIOD_MS    (300u)
// 贴墙前进超时时间，ms（设为0表示不超时）
#define BASE_FOLLOW_TIMEOUT_MS        (0u)
// 贴墙默认前向减速区间，距离前方停止线小于该值后按距离线性降低前进速度
#define BASE_FOLLOW_DEFAULT_SLOW_DIST_MM    (80u)
// 前方距离停车时提前进入减速区，减小到达停止线时的急停冲击
#define BASE_FRONT_STOP_SLOW_DIST_MM        (200u)
// 前方距离停车时的最低速度比例，过低容易卡住，过高停车冲击大
#define BASE_FRONT_STOP_MIN_SPEED_PERCENT   (60u)
// 前方雷达有效距离范围，过滤 0、负数和明显越界毛刺
#define BASE_FRONT_LIDAR_MIN_VALID_MM       (50u)
#define BASE_FRONT_LIDAR_MAX_VALID_MM       (6000u)
// TODO: 实车确认雷达 confidence 范围后，可把该阈值从 0 调高
#define BASE_FRONT_LIDAR_MIN_CONFIDENCE     (0u)
// 前方停止必须由不同时间戳的连续有效帧确认，避免单帧毛刺误退出
#define BASE_FRONT_STOP_CONFIRM_COUNT       (1u)
// 贴墙最小前进速度比例，避免到目标侧距附近完全停住
#define BASE_FOLLOW_MIN_SPEED_PERCENT       (35u)
// I项最大只允许占当前差速修正能力的比例，防止积分过大导致贴墙越修越偏
#define BASE_FOLLOW_I_TERM_LIMIT_PERCENT    (35u)

volatile int16_t g_base_front_lidar_raw_distance_mm = 0;
volatile uint8_t g_base_front_lidar_status = 0u;
volatile uint32_t g_base_front_lidar_timestamp = 0u;
volatile uint8_t g_base_front_lidar_valid = 0u;
volatile uint8_t g_base_front_lidar_stop_confirm_count = 0u;

static float Base_AbsFloat(float value)
{
		return (value >= 0.0f) ? value : -value;
}

// RTOS感知延时，RTOS运行时用osDelay，否则用HAL_Delay
static void Base_DelayMs(uint32_t delay_ms)
{
		if (delay_ms == 0u)
				return;
		if (osKernelGetState() == osKernelRunning)
		{
				osDelay(delay_ms);
		}
		else
		{
				HAL_Delay(delay_ms);
		}
}

static uint8_t Base_ReadFrontLidarDistance(uint16_t *distance_mm_out, uint32_t *timestamp_out)
{
		LidarSensor_t *front_lidar = &g_LidarArray[1];
		int16_t distance_mm;

		if ((distance_mm_out == NULL) || (timestamp_out == NULL))
		{
				return 0u;
		}

		distance_mm = front_lidar->points[0].distance;
		g_base_front_lidar_raw_distance_mm = distance_mm;
		g_base_front_lidar_status = front_lidar->status;
		g_base_front_lidar_timestamp = front_lidar->timestamp;
		g_base_front_lidar_valid = 0u;
		if (front_lidar->status != 0u)
		{
				return 0u;
		}
		if ((distance_mm < (int16_t)BASE_FRONT_LIDAR_MIN_VALID_MM) ||
		    (distance_mm > (int16_t)BASE_FRONT_LIDAR_MAX_VALID_MM))
		{
				return 0u;
		}
#if (BASE_FRONT_LIDAR_MIN_CONFIDENCE > 0u)
		if (front_lidar->points[0].confidence < BASE_FRONT_LIDAR_MIN_CONFIDENCE)
		{
				return 0u;
		}
#endif

		*distance_mm_out = (uint16_t)distance_mm;
		*timestamp_out = front_lidar->timestamp;
		g_base_front_lidar_valid = 1u;
		return 1u;
}

static void Base_SetFrontStopConfirmDebug(uint8_t confirm_count)
{
		g_base_front_lidar_stop_confirm_count = confirm_count;
}

static uint8_t Base_UpdateFrontStopConfirm(uint16_t stop_dist_mm,
                                           uint16_t distance_mm,
                                           uint32_t timestamp,
                                           uint32_t *last_timestamp,
                                           uint8_t *has_last_timestamp,
                                           uint8_t *confirm_count)
{
		if ((stop_dist_mm == 0u) ||
		    (last_timestamp == NULL) ||
		    (has_last_timestamp == NULL) ||
		    (confirm_count == NULL))
		{
				return 0u;
		}

		if ((*has_last_timestamp != 0u) && (timestamp == *last_timestamp))
		{
				Base_SetFrontStopConfirmDebug(*confirm_count);
				return 0u;
		}

		*last_timestamp = timestamp;
		*has_last_timestamp = 1u;

		if (distance_mm <= stop_dist_mm)
		{
				if (*confirm_count < BASE_FRONT_STOP_CONFIRM_COUNT)
				{
						(*confirm_count)++;
				}
		}
		else
		{
				*confirm_count = 0u;
		}

		Base_SetFrontStopConfirmDebug(*confirm_count);
		return (*confirm_count >= BASE_FRONT_STOP_CONFIRM_COUNT) ? 1u : 0u;
}

// 将角度归一化到[-180, 180)
static float Base_Norm180(float deg)
{
	while (deg >= 180.0f) deg -= 360.0f;
	while (deg < -180.0f) deg += 360.0f;
	return deg;
}

static float Base_GetYawRelative(void)
{
	return Base_Norm180(Imu_GetYawRelative());
}

static void Base_StopRotate(void)
{
	Emm_V5_Motor_Control(0, 0, 3);
	Base_DelayMs(300);
}

static int16_t Base_AbsSpeed(int16_t speed)
{
	return (speed >= 0) ? speed : (int16_t)(-speed);
}

static uint32_t Base_DistanceCmToPulse(float distance_cm)
{
	float pulse;

	if (distance_cm <= 0.0f)
		return 0u;

	pulse = distance_cm * BASE_DRIVE_PULSE_PER_CM * BASE_DRIVE_DISTANCE_CALIBRATION;
	if (pulse < 1.0f)
		return 1u;

	return (uint32_t)(pulse + 0.5f);
}

static float Base_PulseToDistanceCm(float pulse)
{
	float pulse_per_cm = BASE_DRIVE_PULSE_PER_CM * BASE_DRIVE_DISTANCE_CALIBRATION;

	if (pulse_per_cm <= 0.0f)
		return 0.0f;

	return pulse / pulse_per_cm;
}

static uint32_t Base_AbsPulseFromSigned(int32_t pulse)
{
	if (pulse < 0)
	{
		return (uint32_t)(-(pulse + 1)) + 1u;
	}

	return (uint32_t)pulse;
}

static int32_t Base_DistanceCmToPositionPulse(float distance_cm)
{
	float abs_cm;
	float pulse_f;
	uint32_t pulse;

	if (distance_cm == 0.0f)
		return 0;

	abs_cm = Base_AbsFloat(distance_cm);
	pulse_f = abs_cm * BASE_DRIVE_POSITION_PULSE_PER_CM;
	if (pulse_f < 1.0f)
		pulse = 1u;
	else
		pulse = (uint32_t)(pulse_f + 0.5f);

	return (distance_cm > 0.0f) ? (int32_t)pulse : -(int32_t)pulse;
}

static uint32_t Base_EstimatePositionWaitMs(uint32_t pulse, uint16_t vel_rpm)
{
	float wait_ms;

	if ((pulse == 0u) || (vel_rpm == 0u))
		return BASE_DRIVE_POS_EXTRA_WAIT_MS;

	wait_ms = ((float)pulse * 60000.0f) /
	          ((float)vel_rpm * BASE_DRIVE_PULSE_PER_REV);

	if (wait_ms < 0.0f)
		wait_ms = 0.0f;

	return (uint32_t)(wait_ms + 0.5f) + BASE_DRIVE_POS_EXTRA_WAIT_MS;
}

int Base_ForwardPulsePosition(int32_t pulse, uint16_t vel_rpm, uint8_t acc)
{
	uint32_t abs_pulse;
	uint8_t dir_12;
	uint8_t dir_34;

	if (vel_rpm == 0u)
		return -1;
	if (pulse == 0)
		return -2;

	abs_pulse = Base_AbsPulseFromSigned(pulse);
	dir_12 = (pulse > 0) ? BASE_DRIVE_FORWARD_DIR_12 : BASE_DRIVE_BACKWARD_DIR_12;
	dir_34 = (pulse > 0) ? BASE_DRIVE_FORWARD_DIR_34 : BASE_DRIVE_BACKWARD_DIR_34;

	/*
	 * 底盘按相对位置模式走指定脉冲：不换算距离，不做陀螺仪闭环。
	 * TODO: 若实车位置模式方向与速度模式不一致，只调整上面的方向宏。
	 */
	Emm_V5_Pos_Control(BASE_DRIVE_MOTOR_1, dir_12, vel_rpm, acc, abs_pulse, false, false);
	Emm_V5_Pos_Control(BASE_DRIVE_MOTOR_2, dir_12, vel_rpm, acc, abs_pulse, false, false);
	Emm_V5_Pos_Control(BASE_DRIVE_MOTOR_3, dir_34, vel_rpm, acc, abs_pulse, false, false);
	Emm_V5_Pos_Control(BASE_DRIVE_MOTOR_4, dir_34, vel_rpm, acc, abs_pulse, false, false);

	/*
	 * 当前 Emm_V5 未接入底盘到位反馈，只能用脉冲和 rpm 估算等待。
	 * acc 较小时实际时间会更长，可先小脉冲低速标定 BASE_DRIVE_POS_EXTRA_WAIT_MS。
	 */
	Base_DelayMs(Base_EstimatePositionWaitMs(abs_pulse, vel_rpm));
	return 0;
}

int Base_ForwardDistanceCmNoYaw(float distance_cm, int16_t vel_rpm, uint8_t acc)
{
	int32_t pulse = Base_DistanceCmToPositionPulse(distance_cm);
	uint16_t vel_abs;

	if (vel_rpm == 0)
		return -1;
	if (pulse == 0)
		return -2;

	vel_abs = (uint16_t)Base_AbsSpeed(vel_rpm);
	return Base_ForwardPulsePosition(pulse, vel_abs, acc);
}

static float Base_RotateSignedError(float current_yaw, float goal_yaw, uint8_t preferred_dir)
{
	float shortest_error = Base_Norm180(goal_yaw - current_yaw);

	if (preferred_dir == BASE_ROTATE_DIR_CW)
	{
		// 顺时针时 yaw 数值减小，返回负误差；越过目标后误差会自动变正用于反向微调
		return -Base_Norm180(current_yaw - goal_yaw);
	}
	if (preferred_dir == BASE_ROTATE_DIR_CCW)
	{
		// 逆时针时 yaw 数值增大，返回正误差；越过目标后误差会自动变负用于反向微调
		return Base_Norm180(goal_yaw - current_yaw);
	}

	return shortest_error;
}

static int Base_RotateToGoalYaw(float goal_yaw, int16_t max_speed_rrp, float kp, float kd, uint8_t preferred_dir)
{
	float last_error = 0.0f;
	uint32_t stable_ms = 0u;
	uint32_t elapsed_ms = 0u;
	int16_t max_speed_abs;

	if (max_speed_rrp == 0)
		return -1;

	if ((preferred_dir != BASE_ROTATE_DIR_CW) &&
		(preferred_dir != BASE_ROTATE_DIR_CCW) &&
		(preferred_dir != BASE_ROTATE_DIR_SHORTEST))
	{
		Base_StopRotate();
		return -2;
	}

	max_speed_abs = Base_AbsSpeed(max_speed_rrp);
	goal_yaw = Base_Norm180(goal_yaw);

	for (;;)
	{
		float current_yaw = Base_GetYawRelative();
		float signed_error = Base_RotateSignedError(current_yaw, goal_yaw, preferred_dir);
		float abs_error = Base_AbsFloat(signed_error);
		float derivative;
		bool rotate_cw;

		if (abs_error <= BASE_ROTATE_STOP_WINDOW_DEG)
		{
			Base_StopRotate();
			Base_DelayMs(BASE_ROTATE_SETTLE_WAIT_MS);

			current_yaw = Base_GetYawRelative();
			signed_error = Base_RotateSignedError(current_yaw, goal_yaw, preferred_dir);
			abs_error = Base_AbsFloat(signed_error);

			if (abs_error <= BASE_ROTATE_YAW_TOLERANCE_DEG)
			{
				stable_ms += BASE_ROTATE_SETTLE_WAIT_MS;
				if (stable_ms >= BASE_ROTATE_STABLE_TIME_MS)
					break;
			}
			else if (abs_error <= BASE_ROTATE_STOP_WINDOW_DEG)
			{
				/*
				 * 实车在目标附近常会因惯性和IMU抖动反复越界。
				 * 已进入停车窗口时直接接受，可避免左右微动很久不退出。
				 */
				break;
			}
			else
			{
				stable_ms = 0u;
			}

			last_error = signed_error;
			elapsed_ms += BASE_ROTATE_SETTLE_WAIT_MS;
		#if (BASE_ROTATE_TIMEOUT_MS > 0u)
			if (elapsed_ms >= BASE_ROTATE_TIMEOUT_MS)
			{
				Base_StopRotate();
				return -3;
			}
		#endif
			continue;
		}

		stable_ms = 0u;
		derivative = signed_error - last_error;
		rotate_cw = (signed_error < 0.0f);

		{
			float speed_cmd = kp * abs_error + kd * Base_AbsFloat(derivative);
			float speed_limit = (float)max_speed_abs;
			int16_t speed_out;

			if (abs_error < BASE_ROTATE_SLOW_ZONE_DEG)
			{
				float ratio = abs_error / BASE_ROTATE_SLOW_ZONE_DEG;
				speed_limit = (float)BASE_ROTATE_MIN_SPEED_RRP +
							  ((float)max_speed_abs - (float)BASE_ROTATE_MIN_SPEED_RRP) * ratio;
			}

			if (abs_error < BASE_ROTATE_FINE_ZONE_DEG &&
				speed_limit > (float)BASE_ROTATE_FINE_MAX_SPEED_RRP)
			{
				speed_limit = (float)BASE_ROTATE_FINE_MAX_SPEED_RRP;
			}

			if (speed_cmd > speed_limit)
				speed_cmd = speed_limit;
			if (speed_cmd < (float)BASE_ROTATE_MIN_SPEED_RRP)
				speed_cmd = (float)BASE_ROTATE_MIN_SPEED_RRP;

			speed_out = (int16_t)(speed_cmd + 0.5f);

			// 顺时针：左轮正右轮负，逆时针：左轮负右轮正
			if (rotate_cw)
			{
				Emm_V5_Motor_Control(speed_out, (int16_t)-speed_out, 3);
				Base_DelayMs(3);
			}
			else
			{
				Emm_V5_Motor_Control((int16_t)-speed_out, speed_out, 3);
				Base_DelayMs(3);
			}
		}

		if ((abs_error <= BASE_ROTATE_YAW_TOLERANCE_DEG) && (Base_AbsFloat(derivative) <= BASE_ROTATE_DERR_TOL))
			stable_ms = BASE_ROTATE_STABLE_TIME_MS;

		last_error = signed_error;
		Base_DelayMs(BASE_ROTATE_CTRL_PERIOD_MS);
		elapsed_ms += BASE_ROTATE_CTRL_PERIOD_MS;
	#if (BASE_ROTATE_TIMEOUT_MS > 0u)
		if (elapsed_ms >= BASE_ROTATE_TIMEOUT_MS)
		{
			Base_StopRotate();
			return -3;
		}
	#endif
	}

	Base_StopRotate();
	return 0;
}

/**
 * @brief  闭环控制底盘旋转到指定yaw角度
 * @param  target_yaw   目标相对角度（度），正数=顺时针，负数=逆时针
 * @param  max_speed_rrp 最大轮子速度（rrp）
 * @param  kp           PD控制器P参数
 * @param  kd           PD控制器D参数
 * @retval 0=成功，负值=失败或超时
 *
 * 说明：
 *   - 该函数会持续读取本次 MCU 复位后的相对 yaw，实时调整轮子速度，直到接近目标角度后退出
 *   - 旋转时通过Emm_V5_Motor_Control(左轮, 右轮, 3)实现，正负号决定方向
 *   - 进入目标角度窗口后需连续多次判定通过才停止，防止偶发抖动
 *   - 超时自动停止
 */
int Base_RotateToYaw(float target_yaw, int16_t max_speed_rrp, float kp, float kd)
{
		// 将输入角度视为相对当前车头的旋转量，按方向生成目标控制角度（带回绕处理）
		float start_yaw = Base_GetYawRelative();
		float delta_yaw = (target_yaw >= 0.0f) ? target_yaw : -target_yaw;
		uint8_t direction = (target_yaw >= 0.0f) ? BASE_ROTATE_DIR_CW : BASE_ROTATE_DIR_CCW;
		float goal_yaw;

		if (max_speed_rrp == 0)
				return -1;

		if (direction == BASE_ROTATE_DIR_CW)
		{
			// 顺时针时yaw数值减小
			goal_yaw = Base_Norm180(start_yaw - delta_yaw);
		}
		else if (direction == BASE_ROTATE_DIR_CCW)
		{
			goal_yaw = Base_Norm180(start_yaw + delta_yaw);
		}
		else
		{
			Base_StopRotate();
			return -2;
		}

		return Base_RotateToGoalYaw(goal_yaw, max_speed_rrp, kp, kd, direction);
}

/**
 * @brief  闭环控制底盘旋转到相对 MCU 复位零点的 yaw 角度
 * @param  target_yaw_abs 目标 yaw 角度（度），相对本次 MCU 复位零点，范围建议为 -180 到 +180，超出会自动归一化
 * @param  max_speed_rrp  最大轮子速度（rrp）
 * @param  kp             PD控制器P参数
 * @param  kd             PD控制器D参数
 * @retval 0=成功，负值=失败或超时
 *
 * 说明：
 *   - USART3 收到第一帧有效 IMU yaw 后会作为 0 度，可直接传入相对该零点的目标角度。
 *   - 函数按最短角度方向旋转，例如当前170度、目标-170度，会旋转20度跨过边界。
 */
int Base_RotateToAbsYaw(float target_yaw_abs, int16_t max_speed_rrp, float kp, float kd)
{
	return Base_RotateToGoalYaw(Base_Norm180(target_yaw_abs),
								max_speed_rrp,
								kp,
								kd,
								BASE_ROTATE_DIR_SHORTEST);
}

/**
 * @brief  贴墙前进并保持侧向距离
 * @param  target_dist_mm     目标距离（mm），即期望底盘与墙壁保持的距离
 * @param  stop_dist_mm       前向避障距离（mm），正前方雷达(dist1)小于该值则紧急退出
 * @param  forward_speed_rrp  基准前进速度（rrp，正数=前进）
 * @param  follow_side        贴墙侧选择：BASE_FOLLOW_SIDE_LEFT(左侧贴墙) / BASE_FOLLOW_SIDE_RIGHT(右侧贴墙)
 * @param  kp                 PD控制器比例参数（决定修正的力度）
 * @param  kd                 PD控制器微分参数（决定修正的阻尼，防震荡）
 * @retval 0=成功，负值=失败或超时 (-1:速度为0, -2:贴墙方向错误, -3:超时, -4:遇到障碍物)
 */
int Base_FollowWall(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float kd)
{
        return Base_FollowWallWithSlowdown(target_dist_mm,
                                           stop_dist_mm,
                                           forward_speed_rrp,
                                           follow_side,
                                           kp,
                                           kd,
                                           BASE_FOLLOW_DEFAULT_SLOW_DIST_MM);
}

int Base_FollowWallWithSlowdown(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float kd, uint16_t slow_dist_mm)
{
        return Base_FollowWallPidWithSlowdown(target_dist_mm,
                                             stop_dist_mm,
                                             forward_speed_rrp,
                                             follow_side,
                                             kp,
                                             0.0f,
                                             kd,
                                             slow_dist_mm);
}

int Base_FollowWallPidWithSlowdown(uint16_t target_dist_mm, uint16_t stop_dist_mm, int16_t forward_speed_rrp, uint8_t follow_side, float kp, float ki, float kd, uint16_t slow_dist_mm)
{
        // 记录上一次的误差，用于计算PD控制中的微分项(D项)
        float last_error = 0.0f;
        // 积分误差用于消除长期偏差，下面会做限幅防止积分饱和
        float integral_error = 0.0f;
        // 记录函数运行的累计时间，用于超时判定
        uint32_t elapsed_ms = 0u;
        // 速度绝对值上限，防止算出的补偿速度过大导致失控
        int16_t max_speed_abs;
        int16_t forward_dir;
        int16_t min_speed_abs;
        uint32_t front_stop_last_timestamp = 0u;
        uint8_t front_stop_has_timestamp = 0u;
        uint8_t front_stop_confirm_count = 0u;

        // 【参数合法性检查】
        // 1. 基准速度不能为0，否则车不会走
        if (forward_speed_rrp == 0)
                return -1;
        // 2. 贴墙方向只能是左或右
        if ((follow_side != BASE_FOLLOW_SIDE_LEFT) && (follow_side != BASE_FOLLOW_SIDE_RIGHT))
                return -2;

        // 计算最大速度的绝对值，作为限幅基准
        max_speed_abs = (forward_speed_rrp > 0) ? forward_speed_rrp : (int16_t)(-forward_speed_rrp);
        forward_dir = (forward_speed_rrp > 0) ? 1 : -1;
        min_speed_abs = (int16_t)(((uint32_t)max_speed_abs * BASE_FOLLOW_MIN_SPEED_PERCENT) / 100u);
        if (min_speed_abs < 1)
                min_speed_abs = 1;

        // 进入闭环控制主循环
        for (;;)
        {
                // 【1. 读取传感器数据】
                // g_LidarArray假设是全局雷达数据数组。此处提取三个方向的最新距离（单位：mm）
                uint16_t dist0 = (uint16_t)g_LidarArray[0].points[0].distance; // 右侧雷达距离
                uint16_t dist2 = (uint16_t)g_LidarArray[2].points[0].distance; // 左侧雷达距离
                uint16_t current_dist = (follow_side == BASE_FOLLOW_SIDE_LEFT) ? dist2 : dist0;
                uint16_t front_dist_mm = 0u;
                uint32_t front_timestamp = 0u;
                uint8_t front_dist_valid = Base_ReadFrontLidarDistance(&front_dist_mm, &front_timestamp);
                float forward_speed_cmd = (float)forward_speed_rrp;
                float forward_speed_abs = (float)max_speed_abs;
                float error;
                float derivative;
                float i_term = 0.0f;
                float delta;
                float max_delta;

                if (front_dist_valid == 0u)
                {
                        front_stop_confirm_count = 0u;
                        Base_SetFrontStopConfirmDebug(front_stop_confirm_count);
                }
                else if (Base_UpdateFrontStopConfirm(stop_dist_mm,
                                                     front_dist_mm,
                                                     front_timestamp,
                                                     &front_stop_last_timestamp,
                                                     &front_stop_has_timestamp,
                                                     &front_stop_confirm_count) != 0u)
                {
                        Emm_V5_Motor_Control(0, 0, 3);
                        Base_DelayMs(3);
                        return -4;
                }

                if ((front_dist_valid != 0u) && (stop_dist_mm > 0u) && (slow_dist_mm > 0u))
                {
                        uint16_t remain_mm = (front_dist_mm > stop_dist_mm) ? (uint16_t)(front_dist_mm - stop_dist_mm) : 0u;

                        if (remain_mm < slow_dist_mm)
                        {
                                float ratio = (float)remain_mm / (float)slow_dist_mm;
                                forward_speed_abs = (float)min_speed_abs + ((float)max_speed_abs - (float)min_speed_abs) * ratio;
                                forward_speed_cmd = forward_speed_abs * (float)forward_dir;
                        }
                }

                error = (float)current_dist - (float)target_dist_mm;
                max_delta = Base_AbsFloat(forward_speed_cmd);
                derivative = error - last_error;

                if (ki > 0.0f)
                {
                        float i_limit = (max_delta * (float)BASE_FOLLOW_I_TERM_LIMIT_PERCENT) / 100.0f;

                        if (((error > 0.0f) && (last_error < 0.0f)) ||
                            ((error < 0.0f) && (last_error > 0.0f)))
                        {
                                integral_error = 0.0f;
                        }

                        integral_error += error;
                        i_term = ki * integral_error;

                        if (i_term > i_limit)
                        {
                                i_term = i_limit;
                                integral_error = i_limit / ki;
                        }
                        if (i_term < -i_limit)
                        {
                                i_term = -i_limit;
                                integral_error = -i_limit / ki;
                        }
                }
                else
                {
                        integral_error = 0.0f;
                }

                delta = kp * error + i_term + kd * derivative;
                // 对补偿量 delta 进行限幅，防止计算出的补偿值飞升导致原地打转
                if (delta > max_delta)
                    delta = max_delta;
                if (delta < -max_delta)
                    delta = -max_delta;

                // 【4. 左右轮速度分配（差速转向核心逻辑）】
                // 初始化左右轮速度为基准前进速度
                float left_speed = forward_speed_cmd;
                float right_speed = forward_speed_cmd;

                // 假设此时离墙太远 (error > 0, delta 为正数)
                if (follow_side == BASE_FOLLOW_SIDE_LEFT)
                {
                        // 【左侧贴墙】离左墙太远时需要向左转：
                        // 左轮减速 (-delta)，右轮加速 (+delta) -> 车头偏左，靠近墙壁
                        left_speed -= delta;
                        right_speed += delta;
                }
                else
                {
                        // 【右侧贴墙】离右墙太远时需要向右转：
                        // 左轮加速 (+delta)，右轮减速 (-delta) -> 车头偏右，靠近墙壁
                        left_speed += delta;
                        right_speed -= delta;
                }

                // 【5. 最终速度限幅】
                // 确保叠加了补偿量之后的单个车轮速度，绝不超出硬件/物理设定的最大上限和下限
                if (left_speed > (float)max_speed_abs)
                    left_speed = (float)max_speed_abs;
                if (left_speed < (float)-max_speed_abs)
                    left_speed = (float)-max_speed_abs;
                if (right_speed > (float)max_speed_abs)
                    right_speed = (float)max_speed_abs;
                if (right_speed < (float)-max_speed_abs)
                    right_speed = (float)-max_speed_abs;

                // 【6. 下发指令到底层硬件】
                // 参数3为电机ID或广播ID，由Emm_V5协议决定
                Emm_V5_Motor_Control((int16_t)left_speed, (int16_t)right_speed, 3);
                Base_DelayMs(3); // 留出通信处理时间

                // 【7. 状态更新与周期延时】
                last_error = error; // 保存当前误差，留给下一个周期的微分项计算使用
                Base_DelayMs(BASE_FOLLOW_CTRL_PERIOD_MS); // 维持固定的控制频率（如10ms）
                elapsed_ms += BASE_FOLLOW_CTRL_PERIOD_MS; // 累加运行时间

                // 【8. 超时保护逻辑】
                // 宏定义 BASE_FOLLOW_TIMEOUT_MS > 0 时编译此段代码
            #if (BASE_FOLLOW_TIMEOUT_MS > 0u)
                if (elapsed_ms >= BASE_FOLLOW_TIMEOUT_MS)
                {
                        Emm_V5_Motor_Control(0, 0, 3); // 超时停机
                        Base_DelayMs(3);
                        return -3;
                }
            #endif
        }

}

/**
 * @brief  按相对 MCU 复位零点的 yaw 角度闭环前进，直到正前方距离小于阈值
 * @param  stop_dist_mm       前向避障距离（mm），dist1小于等于该值则停止
 * @param  forward_speed_rrp  基准前进速度（rrp，正数=前进，负数=后退）
 * @param  target_yaw_abs     目标 yaw 角度（度），相对本次 MCU 复位零点，范围建议 -180 到 +180
 * @param  kp                 yaw误差P参数
 * @param  kd                 yaw误差D参数
 * @retval 0=成功，负值=失败或超时 (-1:速度为0, -3:超时, -4:遇到障碍物)
 */
int Base_ForwardHoldAbsYaw(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd)
{
        return Base_ForwardHoldAbsYawPidWithSlowdown(stop_dist_mm,
                                                     forward_speed_rrp,
                                                     target_yaw_abs,
                                                     kp,
                                                     0.0f,
                                                     kd,
                                                     BASE_FOLLOW_DEFAULT_SLOW_DIST_MM);
}

int Base_ForwardHoldAbsYawWithSlowdown(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd, uint16_t slow_dist_mm)
{
        return Base_ForwardHoldAbsYawPidWithSlowdown(stop_dist_mm,
                                                     forward_speed_rrp,
                                                     target_yaw_abs,
                                                     kp,
                                                     0.0f,
                                                     kd,
                                                     slow_dist_mm);
}

int Base_ForwardHoldAbsYawPidWithSlowdown(uint16_t stop_dist_mm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float ki, float kd, uint16_t slow_dist_mm)
{
        float last_error = 0.0f;
        float integral_error = 0.0f;
        uint32_t elapsed_ms = 0u;
        int16_t max_speed_abs;
        int16_t forward_dir;
        int16_t min_speed_abs;
        float target_yaw = Base_Norm180(target_yaw_abs);
        uint32_t front_stop_last_timestamp = 0u;
        uint8_t front_stop_has_timestamp = 0u;
        uint8_t front_stop_confirm_count = 0u;

        if (forward_speed_rrp == 0)
                return -1;

        max_speed_abs = Base_AbsSpeed(forward_speed_rrp);
        forward_dir = (forward_speed_rrp > 0) ? 1 : -1;
        min_speed_abs = (int16_t)(((uint32_t)max_speed_abs * BASE_FOLLOW_MIN_SPEED_PERCENT) / 100u);
        if (min_speed_abs < 1)
                min_speed_abs = 1;

        for (;;)
        {
                uint16_t front_dist_mm = 0u;
                uint32_t front_timestamp = 0u;
                uint8_t front_dist_valid = Base_ReadFrontLidarDistance(&front_dist_mm, &front_timestamp);
                float forward_speed_cmd = (float)forward_speed_rrp;
                float error;
                float derivative;
                float i_term = 0.0f;
                float delta;
                float max_delta;
                float wheel_speed_limit;
                float left_speed;
                float right_speed;

                if (front_dist_valid == 0u)
                {
                        front_stop_confirm_count = 0u;
                        Base_SetFrontStopConfirmDebug(front_stop_confirm_count);
                }
                else if (Base_UpdateFrontStopConfirm(stop_dist_mm,
                                                     front_dist_mm,
                                                     front_timestamp,
                                                     &front_stop_last_timestamp,
                                                     &front_stop_has_timestamp,
                                                     &front_stop_confirm_count) != 0u)
                {
                        Emm_V5_Motor_Control(0, 0, 3);
                        Base_DelayMs(3);
                        return -4;
                }

                if ((front_dist_valid != 0u) && (stop_dist_mm > 0u) && (slow_dist_mm > 0u))
                {
                        uint16_t remain_mm = (front_dist_mm > stop_dist_mm) ? (uint16_t)(front_dist_mm - stop_dist_mm) : 0u;

                        if (remain_mm < slow_dist_mm)
                        {
                                float ratio = (float)remain_mm / (float)slow_dist_mm;
                                float forward_speed_abs = (float)min_speed_abs + ((float)max_speed_abs - (float)min_speed_abs) * ratio;
                                forward_speed_cmd = forward_speed_abs * (float)forward_dir;
                        }
                }

                /*
                 * error > 0 表示目标 yaw 在当前 yaw 的逆时针方向：
                 * 左轮减速、右轮加速，让车头向逆时针方向修正。
                 */
                error = Base_Norm180(target_yaw - Base_GetYawRelative());
                derivative = error - last_error;
                max_delta = Base_AbsFloat(forward_speed_cmd);
                wheel_speed_limit = max_delta;

                if (ki > 0.0f)
                {
                        float i_limit = (max_delta * (float)BASE_FOLLOW_I_TERM_LIMIT_PERCENT) / 100.0f;

                        if (((error > 0.0f) && (last_error < 0.0f)) ||
                            ((error < 0.0f) && (last_error > 0.0f)))
                        {
                                integral_error = 0.0f;
                        }

                        integral_error += error;
                        i_term = ki * integral_error;

                        if (i_term > i_limit)
                        {
                                i_term = i_limit;
                                integral_error = i_limit / ki;
                        }
                        if (i_term < -i_limit)
                        {
                                i_term = -i_limit;
                                integral_error = -i_limit / ki;
                        }
                }
                else
                {
                        integral_error = 0.0f;
                }

                delta = kp * error + i_term + kd * derivative;
                if (delta > max_delta)
                        delta = max_delta;
                if (delta < -max_delta)
                        delta = -max_delta;

                left_speed = forward_speed_cmd - delta;
                right_speed = forward_speed_cmd + delta;

                if (left_speed > wheel_speed_limit)
                        left_speed = wheel_speed_limit;
                if (left_speed < -wheel_speed_limit)
                        left_speed = -wheel_speed_limit;
                if (right_speed > wheel_speed_limit)
                        right_speed = wheel_speed_limit;
                if (right_speed < -wheel_speed_limit)
                        right_speed = -wheel_speed_limit;

                Emm_V5_Motor_Control((int16_t)left_speed, (int16_t)right_speed, 3);
                Base_DelayMs(3);

                last_error = error;
                Base_DelayMs(BASE_FOLLOW_CTRL_PERIOD_MS);
                elapsed_ms += BASE_FOLLOW_CTRL_PERIOD_MS;

            #if (BASE_FOLLOW_TIMEOUT_MS > 0u)
                if (elapsed_ms >= BASE_FOLLOW_TIMEOUT_MS)
                {
                        Emm_V5_Motor_Control(0, 0, 3);
                        Base_DelayMs(3);
                        return -3;
                }
            #endif
        }
}

	/**
 * @brief  按相对 MCU 复位零点的 yaw 角度闭环前进，直到正前方距离小于阈值（Base_RotateToGoalYaw风格PD）
 * @param  stop_dist_mm       前向避障距离（mm），dist1小于等于该值则停止
 * @param  forward_speed_rrp  基准前进速度（rrp，正数=前进，负数=后退）
 * @param  target_yaw_abs     目标 yaw 角度（度），相对本次 MCU 复位零点
 * @param  kp                 yaw误差P参数
 * @param  kd                 yaw误差D参数
 * @param  min_run_time_ms    最短前进时间（ms），未达到该时间时忽略前方距离触发
 * @retval 0=成功，-1=速度为0，-4=遇到障碍物
 */
int Base_ForwardUntilFrontDistanceHoldYaw(uint16_t stop_dist_mm,
                                          int16_t forward_speed_rrp,
                                          float target_yaw_abs,
                                          float kp,
                                          float kd,
                                          uint32_t min_run_time_ms)
{
	float last_error = 0.0f;
	uint32_t elapsed_ms = 0u;
	int16_t max_speed_abs;
	int16_t forward_dir;
	int16_t min_fwd_speed_abs;
	float target_yaw = Base_Norm180(target_yaw_abs);
	uint32_t front_stop_last_timestamp = 0u;
	uint8_t front_stop_has_timestamp = 0u;
	uint8_t front_stop_confirm_count = 0u;
	uint8_t front_stop_confirmed = 0u;
	uint32_t start_tick = 0u;

	if (forward_speed_rrp == 0)
		return -1;

	max_speed_abs = Base_AbsSpeed(forward_speed_rrp);
	forward_dir = (forward_speed_rrp > 0) ? 1 : -1;
	min_fwd_speed_abs = (int16_t)(((uint32_t)max_speed_abs * BASE_FRONT_STOP_MIN_SPEED_PERCENT) / 100u);
	if (min_fwd_speed_abs < 1)
		min_fwd_speed_abs = 1;

	start_tick = HAL_GetTick();
	for (;;)
	{
		uint16_t front_dist_mm = 0u;
		uint32_t front_timestamp = 0u;
		uint8_t front_dist_valid = Base_ReadFrontLidarDistance(&front_dist_mm, &front_timestamp);
		float current_yaw = Base_GetYawRelative();
		float error = Base_Norm180(target_yaw - current_yaw);
		float abs_error = Base_AbsFloat(error);
		float derivative;
		float forward_speed_cmd = (float)forward_speed_rrp;
		float forward_speed_abs = (float)max_speed_abs;
		float correction;
		float correction_limit;
		float wheel_speed_limit;
		float left_speed, right_speed;

		elapsed_ms = HAL_GetTick() - start_tick;

		if (front_dist_valid == 0u)
		{
			front_stop_confirm_count = 0u;
			front_stop_confirmed = 0u;
			Base_SetFrontStopConfirmDebug(front_stop_confirm_count);
		}
		else if (Base_UpdateFrontStopConfirm(stop_dist_mm,
		                                    front_dist_mm,
		                                    front_timestamp,
		                                    &front_stop_last_timestamp,
		                                    &front_stop_has_timestamp,
		                                    &front_stop_confirm_count) != 0u)
		{
			front_stop_confirmed = 1u;
		}
		else if ((front_dist_valid != 0u) && (front_dist_mm > stop_dist_mm))
		{
			front_stop_confirmed = 0u;
		}

		if ((front_stop_confirmed != 0u) && (elapsed_ms >= min_run_time_ms))
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(300);
			return 0;
		}

		if ((front_dist_valid != 0u) && (stop_dist_mm > 0u) && (BASE_FRONT_STOP_SLOW_DIST_MM > 0u))
		{
			uint16_t remain_mm = (front_dist_mm > stop_dist_mm) ? (uint16_t)(front_dist_mm - stop_dist_mm) : 0u;

			if (remain_mm < BASE_FRONT_STOP_SLOW_DIST_MM)
			{
				float ratio = (float)remain_mm / (float)BASE_FRONT_STOP_SLOW_DIST_MM;
				forward_speed_abs = (float)min_fwd_speed_abs + ((float)max_speed_abs - (float)min_fwd_speed_abs) * ratio;
				forward_speed_cmd = forward_speed_abs * (float)forward_dir;
			}
		}
		wheel_speed_limit = Base_AbsFloat(forward_speed_cmd);

		derivative = error - last_error;

		/* Base_RotateToGoalYaw style PD: speed = kp*|err| + kd*|deriv|, with slowdown/fine/min zones */
		{
			float speed_cmd = kp * abs_error + kd * Base_AbsFloat(derivative);

			if (abs_error < 0.01f)
			{
				speed_cmd = 0.0f;
				correction_limit = 0.0f;
			}
			else
			{
				correction_limit = wheel_speed_limit;
				if (abs_error < BASE_FWDYAW_SLOW_ZONE_DEG)
				{
					float ratio = abs_error / BASE_FWDYAW_SLOW_ZONE_DEG;
					float min_correction = (wheel_speed_limit < (float)BASE_FWDYAW_MIN_SPEED_RRP) ?
					                       wheel_speed_limit : (float)BASE_FWDYAW_MIN_SPEED_RRP;
					correction_limit = min_correction + (wheel_speed_limit - min_correction) * ratio;
				}
				if (abs_error < BASE_FWDYAW_FINE_ZONE_DEG &&
				    correction_limit > (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP)
				{
					correction_limit = (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP;
				}

				if (speed_cmd > correction_limit)
					speed_cmd = correction_limit;
				if (speed_cmd < (float)BASE_FWDYAW_MIN_SPEED_RRP)
				{
					speed_cmd = (wheel_speed_limit < (float)BASE_FWDYAW_MIN_SPEED_RRP) ?
					            wheel_speed_limit : (float)BASE_FWDYAW_MIN_SPEED_RRP;
				}
			}

			correction = speed_cmd;
		}

		/* error>0: CCW correction needed, left decel right accel */
		if (error > 0.0f)
		{
			left_speed = forward_speed_cmd - correction;
			right_speed = forward_speed_cmd + correction;
		}
		else
		{
			left_speed = forward_speed_cmd + correction;
			right_speed = forward_speed_cmd - correction;
		}

		if (left_speed > (float)max_speed_abs)
			left_speed = (float)max_speed_abs;
		if (left_speed < (float)-max_speed_abs)
			left_speed = (float)-max_speed_abs;
		if (right_speed > (float)max_speed_abs)
			right_speed = (float)max_speed_abs;
		if (right_speed < (float)-max_speed_abs)
			right_speed = (float)-max_speed_abs;
		if (left_speed > wheel_speed_limit)
			left_speed = wheel_speed_limit;
		if (left_speed < -wheel_speed_limit)
			left_speed = -wheel_speed_limit;
		if (right_speed > wheel_speed_limit)
			right_speed = wheel_speed_limit;
		if (right_speed < -wheel_speed_limit)
			right_speed = -wheel_speed_limit;

		Emm_V5_Motor_Control((int16_t)left_speed, (int16_t)right_speed, 3);
		Base_DelayMs(3);

		last_error = error;
		Base_DelayMs(BASE_FWDYAW_CTRL_PERIOD_MS);
		elapsed_ms = HAL_GetTick() - start_tick;

	#if (BASE_FOLLOW_TIMEOUT_MS > 0u)
		if (elapsed_ms >= BASE_FOLLOW_TIMEOUT_MS)
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(3);
			return -3;
		}
	#endif
	}
}

/**
 * @brief  按相对 MCU 复位零点的 yaw 闭环前进，直到正前方距离到达阈值；前进过程中监听 Pi 的 HIT。
 * @param  stop_dist_mm       前向停止距离（mm），dist1 小于等于该值则正常退出。
 * @param  forward_speed_rrp  基准前进速度（rrp，正数=前进，负数=后退）
 * @param  target_yaw_abs     目标 yaw 角度（度），相对本次 MCU 复位零点
 * @param  kp                 yaw误差P参数
 * @param  kd                 yaw误差D参数
 * @param  tree_id            当前果树编号，原样发送给 Pi WATCH 协议。
 * @param  view_id            当前观察方向，原样发送给 Pi WATCH 协议。
 * @retval 0 前方距离到达阈值，正常结束。
 * @retval 1 收到合法 HIT，已经停车并停止 WATCH，调用方应 SCAN 重拍。
 * @retval -1 速度为 0。
 * @retval -3 底盘控制超时。
 * @retval 其他负数 Pi 通信错误或参数错误。
 */
int Base_ForwardUntilFrontDistanceHoldYawWatchPi(uint16_t stop_dist_mm,
                                                 int16_t forward_speed_rrp,
                                                 float target_yaw_abs,
                                                 float kp,
                                                 float kd,
                                                 uint8_t tree_id,
                                                 TreeViewId_t view_id,
                                                 uint32_t min_run_time_ms)
{
	uint16_t watch_seq = 0u;
	FruitTarget_t hit = {FRUIT_BIG, 0, 0, 0, 0, 0};
	float last_error = 0.0f;
	uint32_t elapsed_ms = 0u;
	int16_t max_speed_abs;
	int16_t forward_dir;
	int16_t min_fwd_speed_abs;
	float target_yaw = Base_Norm180(target_yaw_abs);
	int pi_ret;
	uint32_t front_stop_last_timestamp = 0u;
	uint8_t front_stop_has_timestamp = 0u;
	uint8_t front_stop_confirm_count = 0u;
	uint8_t front_stop_confirmed = 0u;
	uint32_t start_tick = 0u;

	if (forward_speed_rrp == 0)
		return -1;

	pi_ret = Pi_StartWatch(tree_id, view_id, &watch_seq, 1000u);
	if (pi_ret != PI_OK)
		return pi_ret;

	max_speed_abs = Base_AbsSpeed(forward_speed_rrp);
	forward_dir = (forward_speed_rrp > 0) ? 1 : -1;
	min_fwd_speed_abs = (int16_t)(((uint32_t)max_speed_abs * BASE_FRONT_STOP_MIN_SPEED_PERCENT) / 100u);
	if (min_fwd_speed_abs < 1)
		min_fwd_speed_abs = 1;

	start_tick = HAL_GetTick();
	for (;;)
	{
		uint16_t front_dist_mm = 0u;
		uint32_t front_timestamp = 0u;
		uint8_t front_dist_valid = Base_ReadFrontLidarDistance(&front_dist_mm, &front_timestamp);
		float current_yaw = Base_GetYawRelative();
		float error = Base_Norm180(target_yaw - current_yaw);
		float abs_error = Base_AbsFloat(error);
		float derivative;
		float forward_speed_cmd = (float)forward_speed_rrp;
		float forward_speed_abs = (float)max_speed_abs;
		float correction;
		float correction_limit;
		float wheel_speed_limit;
		float left_speed, right_speed;

		elapsed_ms = HAL_GetTick() - start_tick;

		pi_ret = Pi_PollWatchHit(watch_seq, &hit, 20u);
		if (pi_ret == PI_OK)
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(300);
			(void)Pi_StopWatch(watch_seq);
			return 1;
		}
		if ((pi_ret != PI_ERR_TIMEOUT) && (pi_ret != PI_ERR_OUT_OF_RANGE))
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(300);
			(void)Pi_StopWatch(watch_seq);
			return pi_ret;
		}

		if (front_dist_valid == 0u)
		{
			front_stop_confirm_count = 0u;
			front_stop_confirmed = 0u;
			Base_SetFrontStopConfirmDebug(front_stop_confirm_count);
		}
		else if (Base_UpdateFrontStopConfirm(stop_dist_mm,
		                                    front_dist_mm,
		                                    front_timestamp,
		                                    &front_stop_last_timestamp,
		                                    &front_stop_has_timestamp,
		                                    &front_stop_confirm_count) != 0u)
		{
			front_stop_confirmed = 1u;
		}
		else if ((front_dist_valid != 0u) && (front_dist_mm > stop_dist_mm))
		{
			front_stop_confirmed = 0u;
		}

		if ((front_stop_confirmed != 0u) && (elapsed_ms >= min_run_time_ms))
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(300);
			(void)Pi_StopWatch(watch_seq);
			return 0;
		}

		if ((front_dist_valid != 0u) && (stop_dist_mm > 0u) && (BASE_FRONT_STOP_SLOW_DIST_MM > 0u))
		{
			uint16_t remain_mm = (front_dist_mm > stop_dist_mm) ? (uint16_t)(front_dist_mm - stop_dist_mm) : 0u;

			if (remain_mm < BASE_FRONT_STOP_SLOW_DIST_MM)
			{
				float ratio = (float)remain_mm / (float)BASE_FRONT_STOP_SLOW_DIST_MM;
				forward_speed_abs = (float)min_fwd_speed_abs + ((float)max_speed_abs - (float)min_fwd_speed_abs) * ratio;
				forward_speed_cmd = forward_speed_abs * (float)forward_dir;
			}
		}
		wheel_speed_limit = Base_AbsFloat(forward_speed_cmd);

		derivative = error - last_error;

		{
			float speed_cmd = kp * abs_error + kd * Base_AbsFloat(derivative);

			if (abs_error < 0.01f)
			{
				speed_cmd = 0.0f;
				correction_limit = 0.0f;
			}
			else
			{
				correction_limit = wheel_speed_limit;
				if (abs_error < BASE_FWDYAW_SLOW_ZONE_DEG)
				{
					float ratio = abs_error / BASE_FWDYAW_SLOW_ZONE_DEG;
					float min_correction = (wheel_speed_limit < (float)BASE_FWDYAW_MIN_SPEED_RRP) ?
					                       wheel_speed_limit : (float)BASE_FWDYAW_MIN_SPEED_RRP;
					correction_limit = min_correction + (wheel_speed_limit - min_correction) * ratio;
				}
				if (abs_error < BASE_FWDYAW_FINE_ZONE_DEG &&
				    correction_limit > (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP)
				{
					correction_limit = (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP;
				}

				if (speed_cmd > correction_limit)
					speed_cmd = correction_limit;
				if (speed_cmd < (float)BASE_FWDYAW_MIN_SPEED_RRP)
				{
					speed_cmd = (wheel_speed_limit < (float)BASE_FWDYAW_MIN_SPEED_RRP) ?
					            wheel_speed_limit : (float)BASE_FWDYAW_MIN_SPEED_RRP;
				}
			}

			correction = speed_cmd;
		}

		if (error > 0.0f)
		{
			left_speed = forward_speed_cmd - correction;
			right_speed = forward_speed_cmd + correction;
		}
		else
		{
			left_speed = forward_speed_cmd + correction;
			right_speed = forward_speed_cmd - correction;
		}

		if (left_speed > (float)max_speed_abs)
			left_speed = (float)max_speed_abs;
		if (left_speed < (float)-max_speed_abs)
			left_speed = (float)-max_speed_abs;
		if (right_speed > (float)max_speed_abs)
			right_speed = (float)max_speed_abs;
		if (right_speed < (float)-max_speed_abs)
			right_speed = (float)-max_speed_abs;
		if (left_speed > wheel_speed_limit)
			left_speed = wheel_speed_limit;
		if (left_speed < -wheel_speed_limit)
			left_speed = -wheel_speed_limit;
		if (right_speed > wheel_speed_limit)
			right_speed = wheel_speed_limit;
		if (right_speed < -wheel_speed_limit)
			right_speed = -wheel_speed_limit;

		Emm_V5_Motor_Control((int16_t)left_speed, (int16_t)right_speed, 3);
		Base_DelayMs(3);

		last_error = error;
		Base_DelayMs(BASE_FWDYAW_CTRL_PERIOD_MS);
		elapsed_ms = HAL_GetTick() - start_tick;

	#if (BASE_FOLLOW_TIMEOUT_MS > 0u)
		if (elapsed_ms >= BASE_FOLLOW_TIMEOUT_MS)
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(3);
			(void)Pi_StopWatch(watch_seq);
			return -3;
		}
	#endif
	}
}

static int Base_ForwardPulseHoldYawInternal(uint32_t target_pulse,
                                            int16_t forward_speed_rrp,
                                            float target_yaw_abs,
                                            float kp,
                                            float kd)
{
	float last_error = 0.0f;
	float traveled_pulse = 0.0f;
	uint32_t elapsed_ms = 0u;
	int16_t max_speed_abs;
	int16_t forward_dir;
	int16_t min_speed_abs;
	uint32_t slow_pulse;
	float target_yaw = Base_Norm180(target_yaw_abs);

	if (forward_speed_rrp == 0)
		return -1;
	if (target_pulse == 0u)
		return -2;

	max_speed_abs = Base_AbsSpeed(forward_speed_rrp);
	forward_dir = (forward_speed_rrp > 0) ? 1 : -1;
	min_speed_abs = (int16_t)(((uint32_t)max_speed_abs * BASE_DRIVE_STOP_MIN_SPEED_PERCENT) / 100u);
	if (min_speed_abs < 1)
		min_speed_abs = 1;
	slow_pulse = Base_DistanceCmToPulse(BASE_DRIVE_STOP_SLOW_ZONE_CM);
	if (slow_pulse == 0u)
		slow_pulse = 1u;

	while (traveled_pulse < (float)target_pulse)
	{
		float current_yaw = Base_GetYawRelative();
		float error = Base_Norm180(target_yaw - current_yaw);
		float abs_error = Base_AbsFloat(error);
		float derivative = error - last_error;
		float forward_speed_cmd = (float)forward_speed_rrp;
		float remain_pulse = (float)target_pulse - traveled_pulse;
		float forward_speed_abs = (float)max_speed_abs;
		float correction;
		float correction_limit;
		float wheel_speed_limit;
		float left_speed, right_speed;
		float forward_component_abs;

		if (remain_pulse < (float)slow_pulse)
		{
			float ratio = remain_pulse / (float)slow_pulse;
			forward_speed_abs = (float)min_speed_abs +
			                    ((float)max_speed_abs - (float)min_speed_abs) * ratio;
			forward_speed_cmd = forward_speed_abs * (float)forward_dir;
		}
		wheel_speed_limit = Base_AbsFloat(forward_speed_cmd);

		/* Base_RotateToGoalYaw style PD: speed = kp*|err| + kd*|deriv|, with slowdown/fine/min zones */
		{
			float speed_cmd = kp * abs_error + kd * Base_AbsFloat(derivative);

			if (abs_error < 0.01f)
			{
				speed_cmd = 0.0f;
				correction_limit = 0.0f;
			}
			else
			{
				correction_limit = wheel_speed_limit;
				if (abs_error < BASE_FWDYAW_SLOW_ZONE_DEG)
				{
					float ratio = abs_error / BASE_FWDYAW_SLOW_ZONE_DEG;
					correction_limit = (float)BASE_FWDYAW_MIN_SPEED_RRP +
					                   (wheel_speed_limit - (float)BASE_FWDYAW_MIN_SPEED_RRP) * ratio;
				}
				if (abs_error < BASE_FWDYAW_FINE_ZONE_DEG &&
				    correction_limit > (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP)
				{
					correction_limit = (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP;
				}

				if (speed_cmd > correction_limit)
					speed_cmd = correction_limit;
				if (speed_cmd < (float)BASE_FWDYAW_MIN_SPEED_RRP)
					speed_cmd = (float)BASE_FWDYAW_MIN_SPEED_RRP;
			}

			correction = speed_cmd;
		}

		/* error>0: CCW correction needed, left decel right accel */
		if (error > 0.0f)
		{
			left_speed = forward_speed_cmd - correction;
			right_speed = forward_speed_cmd + correction;
		}
		else
		{
			left_speed = forward_speed_cmd + correction;
			right_speed = forward_speed_cmd - correction;
		}

		if (left_speed > (float)max_speed_abs)
			left_speed = (float)max_speed_abs;
		if (left_speed < (float)-max_speed_abs)
			left_speed = (float)-max_speed_abs;
		if (right_speed > (float)max_speed_abs)
			right_speed = (float)max_speed_abs;
		if (right_speed < (float)-max_speed_abs)
			right_speed = (float)-max_speed_abs;
		if (left_speed > wheel_speed_limit)
			left_speed = wheel_speed_limit;
		if (left_speed < -wheel_speed_limit)
			left_speed = -wheel_speed_limit;
		if (right_speed > wheel_speed_limit)
			right_speed = wheel_speed_limit;
		if (right_speed < -wheel_speed_limit)
			right_speed = -wheel_speed_limit;

		Emm_V5_Motor_Control((int16_t)left_speed, (int16_t)right_speed, 3);
		Base_DelayMs(3);

		forward_component_abs = Base_AbsFloat((left_speed + right_speed) * 0.5f);
		traveled_pulse += forward_component_abs * BASE_DRIVE_PULSE_PER_REV *
		                  ((float)BASE_FWDYAW_CTRL_PERIOD_MS / 60000.0f);

		last_error = error;
		Base_DelayMs(BASE_FWDYAW_CTRL_PERIOD_MS);
		elapsed_ms += BASE_FWDYAW_CTRL_PERIOD_MS;

	#if (BASE_FOLLOW_TIMEOUT_MS > 0u)
		if (elapsed_ms >= BASE_FOLLOW_TIMEOUT_MS)
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(3);
			return -3;
		}
	#endif
	}

	Emm_V5_Motor_Control(0, 0, 3);
	Base_DelayMs(300);
	return 0;
}

/**
 * @brief  按指定距离闭环前进，并保持相对 MCU 复位零点的 yaw 角度
 * @param  distance_cm        目标前进距离（cm），必须大于0
 * @param  forward_speed_rrp  基准前进速度（rrp/rpm，正数=前进，负数=后退）
 * @param  target_yaw_abs     目标 yaw 角度（度），相对本次 MCU 复位零点
 * @param  kp                 yaw误差P参数
 * @param  kd                 yaw误差D参数
 * @retval 0=成功，-1=速度为0，-2=目标距离无效，-3=超时
 *
 * 说明：
 *   - 对外传入单位统一为 cm，内部使用 Base_DistanceCmToPulse() 换算为估算脉冲。
 *   - 当前底层没有解析底盘电机实时位置脉冲，本函数用下发速度和控制周期估算累计脉冲。
 *   - TODO: 后续接入 Emm_V5 当前位置读取后，应改为读取左右轮真实脉冲均值作为退出条件。
 */
int Base_ForwardPulseHoldYaw(float distance_cm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd)
{
	uint32_t target_pulse = Base_DistanceCmToPulse(distance_cm);

	if (target_pulse == 0u)
		return -2;

	return Base_ForwardPulseHoldYawInternal(target_pulse,
	                                        forward_speed_rrp,
	                                        target_yaw_abs,
	                                        kp,
	                                        kd);
}

/**
 * @brief  按指定距离闭环前进，并保持相对 MCU 复位零点的 yaw 角度
 * @param  distance_cm        目标前进距离（cm），必须大于0
 * @param  forward_speed_rrp  基准前进速度（rrp/rpm，正数=前进，负数=后退）
 * @param  target_yaw_abs     目标 yaw 角度（度），相对本次 MCU 复位零点
 * @param  kp                 yaw误差P参数
 * @param  kd                 yaw误差D参数
 * @retval 0=成功，-1=速度为0，-2=目标距离无效，-3=超时
 *
 * 说明：
 *   - 轮子参数按 3200脉冲/圈、轮径150mm 换算，再乘 BASE_DRIVE_DISTANCE_CALIBRATION 修正。
 *   - 当前底层没有解析底盘电机实时位置脉冲，内部仍使用速度积分估算距离。
 */
int Base_ForwardDistanceCmHoldYaw(float distance_cm, int16_t forward_speed_rrp, float target_yaw_abs, float kp, float kd)
{
	return Base_ForwardPulseHoldYaw(distance_cm,
	                                forward_speed_rrp,
	                                target_yaw_abs,
	                                kp,
	                                kd);
}

int Base_ForwardDistanceCmHoldYawWatchPi(float distance_cm,
                                         int16_t forward_speed_rrp,
                                         float target_yaw_abs,
                                         float kp,
                                         float kd,
                                         uint8_t tree_id,
                                         TreeViewId_t view_id,
                                         float *traveled_cm_out)
{
	uint32_t target_pulse = Base_DistanceCmToPulse(distance_cm);
	uint16_t watch_seq = 0u;
	FruitTarget_t hit = {FRUIT_BIG, 0, 0, 0, 0, 0};
	float last_error = 0.0f;
	float traveled_pulse = 0.0f;
	uint32_t elapsed_ms = 0u;
	int16_t max_speed_abs;
	int16_t forward_dir;
	int16_t min_speed_abs;
	uint32_t slow_pulse;
	float target_yaw = Base_Norm180(target_yaw_abs);
	int pi_ret;

	if (traveled_cm_out != NULL)
		*traveled_cm_out = 0.0f;

	if (forward_speed_rrp == 0)
		return -1;
	if (target_pulse == 0u)
		return -2;
	if (traveled_cm_out == NULL)
		return -5;

	pi_ret = Pi_StartWatch(tree_id, view_id, &watch_seq, 1000u);
	if (pi_ret != PI_OK)
		return pi_ret;

	max_speed_abs = Base_AbsSpeed(forward_speed_rrp);
	forward_dir = (forward_speed_rrp > 0) ? 1 : -1;
	min_speed_abs = (int16_t)(((uint32_t)max_speed_abs * BASE_DRIVE_STOP_MIN_SPEED_PERCENT) / 100u);
	if (min_speed_abs < 1)
		min_speed_abs = 1;
	slow_pulse = Base_DistanceCmToPulse(BASE_DRIVE_STOP_SLOW_ZONE_CM);
	if (slow_pulse == 0u)
		slow_pulse = 1u;

	while (traveled_pulse < (float)target_pulse)
	{
		float current_yaw = Base_GetYawRelative();
		float error = Base_Norm180(target_yaw - current_yaw);
		float abs_error = Base_AbsFloat(error);
		float derivative = error - last_error;
		float forward_speed_cmd = (float)forward_speed_rrp;
		float remain_pulse = (float)target_pulse - traveled_pulse;
		float forward_speed_abs = (float)max_speed_abs;
		float correction;
		float correction_limit;
		float wheel_speed_limit;
		float left_speed, right_speed;
		float forward_component_abs;

		pi_ret = Pi_PollWatchHit(watch_seq, &hit, 20u);
		if (pi_ret == PI_OK)
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(300);
			*traveled_cm_out = Base_PulseToDistanceCm(traveled_pulse);
			(void)Pi_StopWatch(watch_seq);
			return 1;
		}
		if ((pi_ret != PI_ERR_TIMEOUT) && (pi_ret != PI_ERR_OUT_OF_RANGE))
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(300);
			*traveled_cm_out = Base_PulseToDistanceCm(traveled_pulse);
			(void)Pi_StopWatch(watch_seq);
			return pi_ret;
		}

		if (remain_pulse < (float)slow_pulse)
		{
			float ratio = remain_pulse / (float)slow_pulse;
			forward_speed_abs = (float)min_speed_abs +
			                    ((float)max_speed_abs - (float)min_speed_abs) * ratio;
			forward_speed_cmd = forward_speed_abs * (float)forward_dir;
		}
		wheel_speed_limit = Base_AbsFloat(forward_speed_cmd);

		{
			float speed_cmd = kp * abs_error + kd * Base_AbsFloat(derivative);

			if (abs_error < 0.01f)
			{
				speed_cmd = 0.0f;
				correction_limit = 0.0f;
			}
			else
			{
				correction_limit = wheel_speed_limit;
				if (abs_error < BASE_FWDYAW_SLOW_ZONE_DEG)
				{
					float ratio = abs_error / BASE_FWDYAW_SLOW_ZONE_DEG;
					correction_limit = (float)BASE_FWDYAW_MIN_SPEED_RRP +
					                   (wheel_speed_limit - (float)BASE_FWDYAW_MIN_SPEED_RRP) * ratio;
				}
				if (abs_error < BASE_FWDYAW_FINE_ZONE_DEG &&
				    correction_limit > (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP)
				{
					correction_limit = (float)BASE_FWDYAW_FINE_MAX_SPEED_RRP;
				}

				if (speed_cmd > correction_limit)
					speed_cmd = correction_limit;
				if (speed_cmd < (float)BASE_FWDYAW_MIN_SPEED_RRP)
					speed_cmd = (float)BASE_FWDYAW_MIN_SPEED_RRP;
			}

			correction = speed_cmd;
		}

		if (error > 0.0f)
		{
			left_speed = forward_speed_cmd - correction;
			right_speed = forward_speed_cmd + correction;
		}
		else
		{
			left_speed = forward_speed_cmd + correction;
			right_speed = forward_speed_cmd - correction;
		}

		if (left_speed > (float)max_speed_abs)
			left_speed = (float)max_speed_abs;
		if (left_speed < (float)-max_speed_abs)
			left_speed = (float)-max_speed_abs;
		if (right_speed > (float)max_speed_abs)
			right_speed = (float)max_speed_abs;
		if (right_speed < (float)-max_speed_abs)
			right_speed = (float)-max_speed_abs;
		if (left_speed > wheel_speed_limit)
			left_speed = wheel_speed_limit;
		if (left_speed < -wheel_speed_limit)
			left_speed = -wheel_speed_limit;
		if (right_speed > wheel_speed_limit)
			right_speed = wheel_speed_limit;
		if (right_speed < -wheel_speed_limit)
			right_speed = -wheel_speed_limit;

		Emm_V5_Motor_Control((int16_t)left_speed, (int16_t)right_speed, 3);
		Base_DelayMs(3);

		forward_component_abs = Base_AbsFloat((left_speed + right_speed) * 0.5f);
		traveled_pulse += forward_component_abs * BASE_DRIVE_PULSE_PER_REV *
		                  ((float)BASE_FWDYAW_CTRL_PERIOD_MS / 60000.0f);
		*traveled_cm_out = Base_PulseToDistanceCm(traveled_pulse);

		last_error = error;
		Base_DelayMs(BASE_FWDYAW_CTRL_PERIOD_MS);
		elapsed_ms += BASE_FWDYAW_CTRL_PERIOD_MS;

	#if (BASE_FOLLOW_TIMEOUT_MS > 0u)
		if (elapsed_ms >= BASE_FOLLOW_TIMEOUT_MS)
		{
			Emm_V5_Motor_Control(0, 0, 3);
			Base_DelayMs(3);
			(void)Pi_StopWatch(watch_seq);
			return -3;
		}
	#endif
	}

	Emm_V5_Motor_Control(0, 0, 3);
	Base_DelayMs(300);
	*traveled_cm_out = Base_PulseToDistanceCm(traveled_pulse);
	(void)Pi_StopWatch(watch_seq);
	return 0;
}

/**
 * @brief  以指定速度前进，直到正前方距离小于阈值
 * @param  stop_dist_mm       前向避障距离（mm），dist1小于该值则停止
 * @param  forward_speed_rrp  前进速度（rrp，正数=前进）
 * @retval 0=成功，负值=失败或超时 (-1:速度为0, -4:遇到障碍物)
 */
int Base_ForwardUntilFrontDistance(uint16_t stop_dist_mm, int16_t forward_speed_rrp)
	{
		uint32_t front_stop_last_timestamp = 0u;
		uint8_t front_stop_has_timestamp = 0u;
		uint8_t front_stop_confirm_count = 0u;

		if (forward_speed_rrp == 0)
			return -1;

		for (;;)
		{
			uint16_t front_dist_mm = 0u;
			uint32_t front_timestamp = 0u;
			uint8_t front_dist_valid = Base_ReadFrontLidarDistance(&front_dist_mm, &front_timestamp);

			if (front_dist_valid == 0u)
			{
				front_stop_confirm_count = 0u;
				Base_SetFrontStopConfirmDebug(front_stop_confirm_count);
			}
			else if (Base_UpdateFrontStopConfirm(stop_dist_mm,
			                                    front_dist_mm,
			                                    front_timestamp,
			                                    &front_stop_last_timestamp,
			                                    &front_stop_has_timestamp,
			                                    &front_stop_confirm_count) != 0u)
			{
				Emm_V5_Motor_Control(0, 0, 3);
				Base_DelayMs(3);
				return -4;
			}

			Emm_V5_Motor_Control(forward_speed_rrp, forward_speed_rrp, 3);
			Base_DelayMs(BASE_FOLLOW_CTRL_PERIOD_MS);
		}
	}
