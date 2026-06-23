/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "button.h"
#include "oled.h"
#include "servo.h"

#include "lidar_manager.h"

#include "spi.h"
#include "sc16is752.h"
#include "sbus.h"
#include "sc16_tasks.h"
#include "usart.h"
#include "Arm.h"

#include <stdbool.h>
#include "Emm_V5.h"
#include "event_groups.h"
#include "Limit.h"
#include "base_control.h"
#include "arm_motion.h"
#include "Usart_to_Pi.h"
#include "pi_protocol.h"
#include "fruit_pick.h"
#include "game_task.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* SC16IS752：接收信号量（由驱动在DMA完成中断中释放） */
SemaphoreHandle_t g_sc16RxSemA = NULL;
SemaphoreHandle_t g_sc16RxSemB = NULL;

/* SC16IS752：事件标志组（用于IRQ中断唤醒处理任务） */
EventGroupHandle_t g_sc16EventGroup = NULL;
#define SC16_EVENT_IRQ_TRIGGERED  (1u << 0)  /* IRQ中断触发标志 */

/*
 * 自动抓取流程调试变量。
 * 用 ST-Link 进入 Debug 后加入 Watch 窗口查看：
 *   g_debug_arm_status      XYZ 初始化/回零结果
 *   g_debug_pi_ping_status  PING 结果，0 表示成功
 *   g_debug_pi_scan_status  SCAN/FRUIT 结果，0 表示成功
 *   g_debug_pick_status     FruitPick_PickOne() 结果，0 表示成功
 *   g_debug_fruit_*         STM32 实际解析到的目标
 *   g_debug_creep_exit_*    连续巡航抓取函数退出码，OLED 第 4 行显示 ERR:<code>
 */
volatile int g_debug_creep_exit_status = 0;
volatile uint8_t g_debug_creep_exit_valid = 0u;

/* Definitions for sc16RxATask */
osThreadId_t sc16RxATaskHandle;
const osThreadAttr_t sc16RxATask_attributes = {
  .name = "sc16RxATask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for sc16RxBTask */
osThreadId_t sc16RxBTaskHandle;
const osThreadAttr_t sc16RxBTask_attributes = {
  .name = "sc16RxBTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartOledTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LED_Init();
  Button_Init();
  Servo_Init();
  Limit_Init();
  PiUart2_Init();
   /* USER CODE END Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* SC16IS752：创建接收信号量，使用二值信号量同步DMA接收完成 */
  g_sc16RxSemA = xSemaphoreCreateBinary();
  g_sc16RxSemB = xSemaphoreCreateBinary();

  /* SC16IS752：创建事件标志组，用于IRQ中断唤醒 */
  g_sc16EventGroup = xEventGroupCreate();

  /* OLED/UI：初始化业务状态同步对象，让OLED任务统一刷新显示。 */
  GameUi_Init();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);

  /* SC16IS752：初始化必须在任务启动前完成（用于打开中断、配置FIFO和波特率） */
  if (g_sc16RxSemA != NULL && g_sc16RxSemB != NULL)
  {
    if (SC16_Init(&g_sc16, &hspi1, 14745600u, g_sc16RxSemA, g_sc16RxSemB) == HAL_OK)
    {
      sc16RxATaskHandle = osThreadNew(StartSc16RxATask, NULL, &sc16RxATask_attributes);
      sc16RxBTaskHandle = osThreadNew(StartSc16RxBTask, NULL, &sc16RxBTask_attributes);
    }
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  *  默认任务：初始化舵机、电机、LED、按钮、OLED、SC16IS752等。
  *  写任务的主流程来控制舵机、电机、LED、按钮、OLED、SC16IS752等。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  ArmMotionStatus_t arm_status = ARM_MOTION_OK;
  ArmPoint_t arm_position = {0.0f, ARM_MOTION_Y_PRE_EXTEND_MM, 0.0f};
  FruitTarget_t fruit = {FRUIT_BIG, 0, 0, 0, 0, 0};
  int pi_status = 0;
  int pick_status = 0;
  int creep_exit_status = 0;

  (void)argument;
  (void)arm_status;
  (void)fruit;
  (void)pi_status;
  (void)pick_status;
  /* 上电复位前，先让大小果篮处于垂直避让状态。 */
  Servo_SetAll_Init();

  //等待陀螺仪稳定，避免OLED显示异常数据
  osDelay(1500);
  Emm_V5_All_Init();
  // Emm_V5_Disable_ID1_4();

  //使能所有电机

  /*
   * 测试代码
  */
  // FruitBasket_DumpBig();
  // Base_ForwardDistanceCmNoYaw(-20.0f, 80, 40);

  // 等待树莓派串口协议可用，并等待人工按键确认后才继续比赛主流程。
  //Game_WaitPiReadyAndUserStart();
  // /*
  //  * XYZ 上电回零：回零完成后，立即把 Y 轴移动到 40mm。
  //  * 后续普通 Arm_MoveToPoint() 调用不允许目标 Y 小于 40mm。
  // */

	arm_status = Arm_HomeXYZ();
	if (arm_status == ARM_MOTION_OK)
	{
	  arm_status = Arm_MoveToPoint(0.0f, ARM_MOTION_Y_PRE_EXTEND_MM, 0.0f);
	  if (arm_status == ARM_MOTION_OK)
	  {
		  Servo_SetFruitBasketsLoad();
	  }
	}


  // /*
  //  * 前往采摘区的赛道动作草稿：
  //  *   前进到前方 300mm 阈值，按相对本次 MCU 复位零点的 0/-90/180/-90/0 度航向分段行驶和转向。
  //  *   下面参数均为已测过的保守值，启用前按实车路线逐段取消注释。
  //  */
  Base_ForwardUntilFrontDistanceHoldYaw(400, 80, 0.0f, 5.0f, 0.0f, 2000);
  Base_RotateToAbsYaw(-90.0f, 55.0f, 0.8f, 0.05f);
  Base_ForwardUntilFrontDistanceHoldYaw(400, 100, -90.0f, 5.0f, 0.0f, 3000);
  Base_RotateToAbsYaw(180.0f, 55.0f, 0.8f, 0.05f);
  Base_ForwardUntilFrontDistanceHoldYaw(700, 100, 180.0f, 5.0f, 0.0f, 2000);
  Base_RotateToAbsYaw(-90.0f, 55.0f, 0.8f, 0.05f);
  Base_ForwardUntilFrontDistanceHoldYaw(760, 100, -90.0f, 5.0f, 0.0f, 6000);
  Base_RotateToAbsYaw(0.0f, 55.0f, 0.8f, 0.05f);

  /*
   * 沿直线采摘果实
  */
    if (arm_status == ARM_MOTION_OK)
    {
      g_debug_creep_exit_valid = 0u;
      creep_exit_status = Game_CreepWatchAndPickUntilFrontDistance(
                            0u,
                            TREE_VIEW_LEFT,
                            700u,
                            17,
                            0.0f,
                            5.0f,
                            0.0f,
                            180*1000);
      g_debug_creep_exit_status = creep_exit_status;
      g_debug_creep_exit_valid = 1u;
    }

      Base_RotateToAbsYaw(90.0f, 55.0f, 0.8f, 0.05f);
      Base_ForwardUntilFrontDistanceHoldYaw(400, 100, 90.0f, 5.0f, 0.0f, 4000);
      Base_RotateToAbsYaw(180.0f, 55.0f, 0.8f, 0.05f);
      Base_ForwardUntilFrontDistanceHoldYaw(250, 80, 180.0f, 5.0f, 0.0f, 2000);
      Base_RotateToAbsYaw(90.0f, 55.0f, 0.8f, 0.05f);
      Base_ForwardUntilFrontDistanceHoldYaw(200, 80, 90.0f, 5.0f, 0.0f, 3000);

      //移动Y轴到较为安全的区域
      if (arm_status == ARM_MOTION_OK)
      {
        arm_status = Arm_GetCurrentPosition(&arm_position);
        if (arm_status == ARM_MOTION_OK)
        {
          arm_status = Arm_MoveToPoint(0.0, 40.0f, 0.0);
        }
      }
      FruitBasket_DumpBig();//倒大苹果
	  Base_ForwardDistanceCmNoYaw(-15.0f, 80, 40);
      Base_RotateToAbsYaw(0.0f, 55.0f, 0.8f, 0.05f);
      Base_ForwardUntilFrontDistanceHoldYaw(280, 80, 0.0f, 5.0f, 0.0f, 1000);
	  Base_RotateToAbsYaw(-90.0f, 55.0f, 0.8f, 0.05f);
      Base_ForwardDistanceCmNoYaw(-40.0f, 80, 40);
      FruitBasket_DumpSmall();//倒小苹果

  /* Infinite loop */
  for(;;)
  {
  	 //电机测试
    //Emm_V5_Motor_Control(0, 0, 3);
    /* 每完成一轮翻转一次LED1作为任务心跳 */
    LED_Toggle(1);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief OLED/UI任务：左侧显示激光测距，右侧显示IMU（20ms刷新且互不重叠）
  */
static uint16_t Oled_FloatAbsTo3Digits(float value)
{
  float abs_v;
  uint16_t out;

  abs_v = (value >= 0.0f) ? value : -value;
  if (abs_v > 999.0f)
  {
    abs_v = 999.0f;
  }

  out = (uint16_t)(abs_v + 0.5f);
  return out;
}

void StartOledTask(void *argument)
{
  HAL_StatusTypeDef oled_ok;
  const char *pi_status_text;
  char err_text[17];
  uint16_t dist0;
  uint16_t dist1;
  uint16_t dist2;
  uint16_t imu_yaw;

  (void)argument;

  osDelay(50);
  oled_ok = OLED_Init();

  if (oled_ok == HAL_OK)
  {
    (void)OLED_Clear();

    /* 左侧列(1~8)给激光，右侧列(9~16)给IMU/舵机，中间留一列空白分隔。 */
    (void)OLED_ClearBuffer();
    (void)OLED_DrawString(1, 1, "L1");
    (void)OLED_DrawString(2, 1, "L2");
    (void)OLED_DrawString(3, 1, "L3");

    (void)OLED_DrawString(1, 13, "Y");

    (void)OLED_Flush();
  }

  for(;;)
  {

    /* 每20ms读取一次激光+IMU并刷新到OLED。 */
    if (oled_ok == HAL_OK)
    {
      // 读取三路激光雷达距离（单位：mm），分别显示在OLED左侧三行
      dist0 = (uint16_t)g_LidarArray[0].points[0].distance; // L1
      dist1 = (uint16_t)g_LidarArray[1].points[0].distance; // L2
      dist2 = (uint16_t)g_LidarArray[2].points[0].distance; // L3

      // 读取IMU姿态角（pitch/roll/yaw），并转为3位整数用于显示
      imu_yaw = Oled_FloatAbsTo3Digits(Imu_GetYawRelative());     // Yaw
      (void)Imu_GetFrameReadyAndClear(); // 清除IMU新帧标志

      // OLED左侧显示激光距离，列4，行1/2/3分别为L1/L2/L3
      (void)OLED_DrawNum(1, 4, dist0, 5); // L1
      (void)OLED_DrawNum(2, 4, dist1, 5); // L2
      (void)OLED_DrawNum(3, 4, dist2, 5); // L3

      (void)OLED_DrawNum(1, 14, imu_yaw, 3);     // Yaw，列14

      if (g_debug_creep_exit_valid != 0u)
      {
        (void)OLED_DrawString(4, 1, "                ");
        (void)snprintf(err_text, sizeof(err_text), "ERR:%d", g_debug_creep_exit_status);
        (void)OLED_DrawString(4, 1, err_text);
      }
      else
      {
        pi_status_text = GameUi_GetPiStatusText();
        if (pi_status_text != NULL)
        {
          (void)OLED_DrawString(4, 1, "                ");
          (void)OLED_DrawString(4, 1, pi_status_text);
        }
      }

      // 刷新OLED，将buffer内容显示到屏幕
      (void)OLED_Flush();
    }

    osDelay(20);
  }
}
/* USER CODE END Application */

