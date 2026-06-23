/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    servo.c
  * @brief   舵机控制模块实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"
#include "tim.h"

/* 外部定时器句柄声明（由CubeMX生成） */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN 0 */

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
} servo_hw_t;

/* 舵机硬件映射表：索引与servo_id_e一一对应 */
static const servo_hw_t s_servo_hw[SERVO_COUNT] =
{
  {&htim3, TIM_CHANNEL_3},  /* SERVO_1: TIM3_CH3 (PB0)  */
  {&htim3, TIM_CHANNEL_4},  /* SERVO_2: TIM3_CH4 (PB1)  */
  {&htim2, TIM_CHANNEL_1},  /* SERVO_3: TIM2_CH1 (PA0)  */
  {&htim2, TIM_CHANNEL_2},  /* SERVO_4: TIM2_CH2 (PA1)  */
  {&htim1, TIM_CHANNEL_1},  /* SERVO_5: TIM1_CH1 (PE9)  */
  {&htim1, TIM_CHANNEL_2},  /* SERVO_6: TIM1_CH2 (PE11) */
  {&htim1, TIM_CHANNEL_3},  /* SERVO_7: TIM1_CH3 (PE13) */
  {&htim1, TIM_CHANNEL_4},  /* SERVO_8: TIM1_CH4 (PE14) */
};

static int8_t Servo_IsValidId(servo_id_e id)
{
  return (id < SERVO_COUNT) ? 1 : 0;
}

/* USER CODE END 0 */

/**
  * @brief  舵机统一初始化
  * @note   启动8路PWM输出，并将初始脉宽设置为1500us
  * @retval None
  */
void Servo_Init(void)
{
  uint8_t i;

  for (i = 0; i < (uint8_t)SERVO_COUNT; i++)
  {
    (void)HAL_TIM_PWM_Start(s_servo_hw[i].htim, s_servo_hw[i].channel);
    __HAL_TIM_SET_COMPARE(s_servo_hw[i].htim, s_servo_hw[i].channel, SERVO_PULSE_MID_US);
  }
}

/**
  * @brief  设置指定舵机脉宽（单位us）
  * @param  id: 舵机ID，取值SERVO_1~SERVO_8
  * @param  pulse_us: 目标脉宽，允许范围500~2500us
  * @retval 0=成功, -1=参数错误
  */
int8_t Servo_SetPulse(servo_id_e id, uint16_t pulse_us)
{
  if (Servo_IsValidId(id) == 0)
  {
    return -1;
  }

  if ((pulse_us < SERVO_PULSE_MIN_US) || (pulse_us > SERVO_PULSE_MAX_US))
  {
    return -1;
  }

  __HAL_TIM_SET_COMPARE(s_servo_hw[id].htim, s_servo_hw[id].channel, pulse_us);

  return 0;
}

/**
  * @brief  设置指定舵机角度（270度舵机）
  * @param  id: 舵机ID，取值SERVO_1~SERVO_8
  * @param  angle: 目标角度，允许范围0.0~270.0度
  * @retval 0=成功, -1=参数错误
  */
int8_t Servo_SetAngle(servo_id_e id, float angle)
{
  float pulse_f;
  uint16_t pulse_us;

  if (Servo_IsValidId(id) == 0)
  {
    return -1;
  }

  if ((angle < SERVO_ANGLE_MIN_DEG) || (angle > SERVO_ANGLE_MAX_DEG))
  {
    return -1;
  }

  /* 线性映射：0deg->500us, 270deg->2500us */
  pulse_f = (float)SERVO_PULSE_MIN_US +
            angle * ((float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / SERVO_ANGLE_MAX_DEG);

  /* 四舍五入到整数微秒 */
  pulse_us = (uint16_t)(pulse_f + 0.5f);

  return Servo_SetPulse(id, pulse_us);
}

/* USER CODE BEGIN 2 */
/* 已确认的末端执行器和果篮舵机角度。 */
#define SERVO_CUTTER_OPEN_DEG              270.0f
#define SERVO_GRIPPER_OPEN_DEG             255.0f
#define SERVO_SORTER_CENTER_DEG            135.0f
#define SERVO_BIG_BASKET_VERTICAL_DEG      45.0f
#define SERVO_BIG_BASKET_LOAD_DEG          135.0f
#define SERVO_SMALL_BASKET_VERTICAL_DEG    215.0f
#define SERVO_SMALL_BASKET_LOAD_DEG        125.0f

void Servo_SetFruitBasketsVertical(void)
{
  (void)Servo_SetAngle(SERVO_4, SERVO_BIG_BASKET_VERTICAL_DEG);
  (void)Servo_SetAngle(SERVO_5, SERVO_SMALL_BASKET_VERTICAL_DEG);
}

void Servo_SetFruitBasketsLoad(void)
{
  (void)Servo_SetAngle(SERVO_4, SERVO_BIG_BASKET_LOAD_DEG);
  (void)Servo_SetAngle(SERVO_5, SERVO_SMALL_BASKET_LOAD_DEG);
}

/* 上电复位前的安全初始化：果篮先垂直避让，等待 XYZ 回零。 */
void Servo_SetAll_Init(void)
{
  (void)Servo_SetAngle(SERVO_1, SERVO_CUTTER_OPEN_DEG);
  (void)Servo_SetAngle(SERVO_2, SERVO_GRIPPER_OPEN_DEG);
  (void)Servo_SetAngle(SERVO_3, SERVO_SORTER_CENTER_DEG);
  Servo_SetFruitBasketsVertical();
}


/* 
 1是剪刀的ID,然后角度270度是完全张开的位置,60度是完全夹紧剪刀
 2是夹爪的ID,然后角度255度是完全张开的位置,100度是完全夹紧夹爪
 3是放果篮摆动的ID,然后角度135度是居中的位置,270度是原大果仓/当前小果暂存区方向,0度是原小果仓/当前大果暂存区方向
 4是原当前小果暂存区的ID，角度135度是接收装载的状态，65度是倒出去的状态，45度是垂直避让
 5是当前大果暂存区的ID，角度125度是接收装载的状态，170度是倒出去的状态，215度是垂直避让
 */
/* USER CODE END 2 */
