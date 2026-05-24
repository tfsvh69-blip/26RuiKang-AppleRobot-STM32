/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    button.c
  * @brief   按键输入模块实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "button.h"

/* 在RTOS运行时优先使用osDelay，避免阻塞Systick临界区；未启动RTOS时退回HAL_Delay */
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */

/* 按键状态记录（用于边界检测） */
static uint8_t button_last_state[4] = {KEY_RELEASED, KEY_RELEASED, KEY_RELEASED, KEY_RELEASED};

static void Button_DelayMs(uint32_t delay_ms)
{
  /* CMSIS-RTOS2: osKernelGetState() 只有在kernel初始化后才可用
     为兼容启动早期调用，这里做最保守的判断。 */
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

/* USER CODE END 0 */

/**
  * @brief  初始化按键模块
  * @retval None
  */
void Button_Init(void)
{
  /* 按键初始化已由MX_GPIO_Init完成，此函数保留用于可能的扩展 */
  button_last_state[0] = KEY_RELEASED;
  button_last_state[1] = KEY_RELEASED;
  button_last_state[2] = KEY_RELEASED;
  button_last_state[3] = KEY_RELEASED;
}

/**
  * @brief  扫描指定按键，返回当前状态（无消抖）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval KEY_PRESSED(1) 或 KEY_RELEASED(0)，错误时返回-1
  */
int Button_Read(uint8_t key)
{
  GPIO_PinState state;

  switch(key)
  {
    case 1:
      state = HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN);
      break;
    case 2:
      state = HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);
      break;
    case 3:
      state = HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN);
      break;
    case 4:
      state = HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN);
      break;
    default:
      return -1;  /* 参数错误 */
  }

  /* 按键为低电平触发（按下时为0，释放时为1） */
  return (state == GPIO_PIN_RESET) ? KEY_PRESSED : KEY_RELEASED;
}

/**
  * @brief  获取按键状态并进行消抖处理
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @param  debounce_ms: 消抖延迟时间（毫秒），建议20-50ms
  * @retval KEY_PRESSED(1) 或 KEY_RELEASED(0)，错误时返回-1
  */
int Button_ReadDebounce(uint8_t key, uint32_t debounce_ms)
{
  int state1, state2;

  /* 参数合理性检查 */
  if(key < 1 || key > 4)
    return -1;

  /* 第一次读取 */
  state1 = Button_Read(key);
  if(state1 < 0)
    return -1;

  /* 延迟20-50ms */
  Button_DelayMs(debounce_ms);

  /* 第二次读取 */
  state2 = Button_Read(key);

  /* 两次读取相同则认为稳定 */
  if(state1 == state2)
    return state1;
  else
    return state1;  /* 防抖失败，返回第一次结果 */
}

/**
  * @brief  检测按键按下事件（仅返回一次）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval 1=检测到按下, 0=未按下，-1=错误
  */
int Button_IsPressedEdge(uint8_t key)
{
  int current_state;
  int edge_detected = 0;

  if(key < 1 || key > 4)
    return -1;

  current_state = Button_Read(key);
  if(current_state < 0)
    return -1;

  /* 检测下降沿（从释放到按下） */
  if(current_state == KEY_PRESSED && button_last_state[key-1] == KEY_RELEASED)
  {
    edge_detected = 1;
  }

  /* 更新状态记录 */
  button_last_state[key-1] = current_state;

  return edge_detected;
}

/**
  * @brief  检测按键释放事件（仅返回一次）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval 1=检测到释放, 0=未释放，-1=错误
  */
int Button_IsReleasedEdge(uint8_t key)
{
  int current_state;
  int edge_detected = 0;

  if(key < 1 || key > 4)
    return -1;

  current_state = Button_Read(key);
  if(current_state < 0)
    return -1;

  /* 检测上升沿（从按下到释放） */
  if(current_state == KEY_RELEASED && button_last_state[key-1] == KEY_PRESSED)
  {
    edge_detected = 1;
  }

  /* 更新状态记录 */
  button_last_state[key-1] = current_state;

  return edge_detected;
}

/**
  * @brief  获取所有按键状态
  * @param  states: 长度至少为4的数组，存储4个按键的状态
  * @retval 0=成功, -1=参数错误
  */
int Button_ReadAll(uint8_t *states)
{
  int i;
  int st;

  if(states == NULL)
    return -1;

  for(i = 0; i < (int)BUTTON_KEY_COUNT; i++)
  {
    st = Button_Read((uint8_t)(i + 1));
    if (st < 0)
      return -1;
    states[i] = (uint8_t)st;
  }

  return 0;
}

/**
  * @brief  等待指定按键被按下（阻塞函数）
  * @param  key: 按键编号 (1, 2, 3, 4)
  * @retval 0=成功, -1=错误
  */
int Button_WaitPress(uint8_t key)
{
  if(key < 1 || key > 4)
    return -1;

  /* 等待按键按下（轮询方式） */
  while(Button_Read(key) != KEY_PRESSED)
  {
    Button_DelayMs(10);  /* 避免频繁轮询 */
  }

  return 0;
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
