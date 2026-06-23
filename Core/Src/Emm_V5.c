#include "Emm_V5.h"
#include "cmsis_os.h"
#include "Emm_V5.h"

// RTOS感知延时，RTOS运行时用osDelay，否则用HAL_Delay
static void Emm_V5_DelayMs(uint32_t delay_ms)
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

/**
 * @brief 电机当前位置清零（复位当前位置为0）
 * @param addr 电机地址
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[4] = {addr, 0x0A, 0x6D, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
  Emm_V5_DelayMs(10);
}

/**
 * @brief 清除堵转保护
 * @param addr 电机地址
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[4] = {addr, 0x0E, 0x52, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief 读取系统参数
 * @param addr 电机地址
 * @param s 参数类型（枚举SysParams_t）
 * @note 发送不同的参数码可读取版本、PID、实时速度、电压等
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  cmd[i++] = addr;
  switch (s)
  {
    case S_VER  : cmd[i++] = 0x1F; break;
    case S_RL   : cmd[i++] = 0x20; break;
    case S_PID  : cmd[i++] = 0x21; break;
    case S_VBUS : cmd[i++] = 0x24; break;
    case S_CPHA : cmd[i++] = 0x27; break;
    case S_ENCL : cmd[i++] = 0x31; break;
    case S_TPOS : cmd[i++] = 0x33; break;
    case S_VEL  : cmd[i++] = 0x35; break;
    case S_CPOS : cmd[i++] = 0x36; break;
    case S_PERR : cmd[i++] = 0x37; break;
    case S_FLAG : cmd[i++] = 0x3A; break;
    case S_ORG  : cmd[i++] = 0x3B; break;
    case S_Conf : cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
    case S_State: cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
    default: break;
  }
  cmd[i++] = 0x6B;

  usart_SendCmd(cmd, i);
}

/**
 * @brief 修改控制模式（开环/闭环/位置/速度等）
 * @param addr 电机地址
 * @param svF 是否存储到EEPROM（true=存储，false=仅本次有效）
 * @param ctrl_mode 控制模式编号
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[6] = {addr, 0x46, 0x69, (uint8_t)svF, ctrl_mode, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief 使能/失能电机
 * @param addr 电机地址
 * @param state true=使能，false=失能
 * @param snF 是否同步（true=同步，false=单独）
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[6] = {addr, 0xF3, 0xAB, (uint8_t)state, (uint8_t)snF, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief 速度模式控制（恒速旋转）
 * @param addr 电机地址
 * @param dir 方向（0=CW顺时针，1=CCW逆时针）
 * @param vel 目标转速（单位RPM）
 * @param acc 加速度（0为立即变速）
 * @param snF 是否同步
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[8] = {
    addr,
    0xF6,
    dir,
    (uint8_t)(vel >> 8),
    (uint8_t)(vel >> 0),
    acc,
    (uint8_t)snF,
    0x6B
  };
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
  osDelay(10);
}

/**
 * @brief 位置模式控制（定点运动）
 * @param addr 电机地址
 * @param dir 方向（0=CW顺时针，1=CCW逆时针）
 * @param vel 目标转速（单位RPM）
 * @param acc 加速度
 * @param clk 目标脉冲数（绝对/相对位置）
 * @param raF 位置类型（0=相对，1=绝对）
 * @param snF 是否同步
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[13] = {
    addr,
    0xFD,
    dir,
    (uint8_t)(vel >> 8),
    (uint8_t)(vel >> 0),
    acc,
    (uint8_t)(clk >> 24),
    (uint8_t)(clk >> 16),
    (uint8_t)(clk >> 8),
    (uint8_t)(clk >> 0),
    (uint8_t)raF,
    (uint8_t)snF,
    0x6B
  };
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
  osDelay(10);
}

/**
 * @brief 立即停止运动
 * @param addr 电机地址
 * @param snF 是否同步
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[5] = {addr, 0xFE, 0x98, (uint8_t)snF, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
  Emm_V5_DelayMs(3);
}

/**
 * @brief 多电机同步启动
 * @param addr 电机地址
 */
void Emm_V5_Synchronous_motion(void)
{
  uint8_t cmd[4] = {0x00, 0xFF, 0x66, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief 设置当前为原点（回零）
 * @param addr 电机地址
 * @param svF 是否存储
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[5] = {addr, 0x93, 0x88, (uint8_t)svF, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
  Emm_V5_DelayMs(10);
}

/**
 * @brief 修改回零参数
 * @param addr 电机地址
 * @param svF 是否存储
 * @param o_mode 回零模式
 * @param o_dir 回零方向
 * @param o_vel 回零速度
 * @param o_tm 回零超时时间
 * @param sl_vel 碰撞速度
 * @param sl_ma 碰撞电流
 * @param sl_ms 碰撞保持时间
 * @param potF 是否启用限位
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir,
                                 uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel,
                                 uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[20] = {
    addr,
    0x4C,
    0xAE,
    (uint8_t)svF,
    o_mode,
    o_dir,
    (uint8_t)(o_vel >> 8),
    (uint8_t)(o_vel >> 0),
    (uint8_t)(o_tm >> 24),
    (uint8_t)(o_tm >> 16),
    (uint8_t)(o_tm >> 8),
    (uint8_t)(o_tm >> 0),
    (uint8_t)(sl_vel >> 8),
    (uint8_t)(sl_vel >> 0),
    (uint8_t)(sl_ma >> 8),
    (uint8_t)(sl_ma >> 0),
    (uint8_t)(sl_ms >> 8),
    (uint8_t)(sl_ms >> 0),
    (uint8_t)potF,
    0x6B
  };
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief 触发回零动作
 * @param addr 电机地址
 * @param o_mode 回零模式
 * @param snF 是否同步
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[5] = {addr, 0x9A, o_mode, (uint8_t)snF, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief 强制中断回零过程
 * @param addr 电机地址
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[4] = {addr, 0x9C, 0x48, 0x6B};
  usart_SendCmd(cmd, (uint8_t)sizeof(cmd));
}
//初始化所有电机
void Emm_V5_All_Init(void)
{
  Emm_V5_En_Control(1, true, false);
  osDelay(50);
  Emm_V5_En_Control(2, true, false);
  osDelay(50);
  Emm_V5_En_Control(3, true, false);
  osDelay(50);
  Emm_V5_En_Control(4, true, false);
  osDelay(50);
  Emm_V5_En_Control(5, true, false);
  osDelay(50);
  Emm_V5_En_Control(6, true, false);
  osDelay(50);
  Emm_V5_En_Control(7, true, false);
  osDelay(50);
	Emm_V5_Stop_Now(1, false);
  osDelay(50);
	Emm_V5_Stop_Now(2, false);
  osDelay(50);
	Emm_V5_Stop_Now(3, false);
  osDelay(50);
	Emm_V5_Stop_Now(4, false);
  osDelay(50);
	Emm_V5_Stop_Now(5, false);
  osDelay(50);
	Emm_V5_Stop_Now(6, false);
  osDelay(50);
	Emm_V5_Stop_Now(7, false);
  osDelay(50);
}

void Emm_V5_Disable_ID1_4(void)
{
  Emm_V5_En_Control(1, false, false);
  osDelay(50);
  Emm_V5_En_Control(2, false, false);
  osDelay(50);
  Emm_V5_En_Control(3, false, false);
  osDelay(50);
  Emm_V5_En_Control(4, false, false);
  osDelay(50);
}

/**
 * @brief 控制4个电机的速度和方向（带反转逻辑和限幅），并依次下发速度指令
 *
 * @param speed_34  输入：原3/4号轮的速度（范围-100~100，正负表示方向）
 * @param speed_12  输入：原1/2号轮的速度（范围-100~100，正负表示方向）
 * @param delay_ms  每次下发指令后的延时（单位ms）
 *
 * 反转逻辑说明：
 *   - 左/右轮互换，前进/后退取反（左轮=原右轮取反，右轮=原左轮取反）
 *   - mapped_12 = (speed_34 == 0) ? 0 : -speed_34
 *   - mapped_34 = (speed_12 == 0) ? 0 : -speed_12
 *
 * 限幅说明：
 *   - 超过±100会被限制到±100
 *
 * 电机方向协议：
 *   - 1/2号电机：0=正转，1=反转
 *   - 3/4号电机：1=正转，0=反转（与1/2号相反）
 */
void Emm_V5_Motor_Control(int16_t speed_34, int16_t speed_12, uint32_t delay_ms)
{
  if ((speed_34 == 0) && (speed_12 == 0))
  {
    Emm_V5_Stop_Now(1, false);
    osDelay(3);
    Emm_V5_Stop_Now(2, false);
    osDelay(3);
    Emm_V5_Stop_Now(3, false);
    osDelay(3);
    Emm_V5_Stop_Now(4, false);
    osDelay(3);
    return;
  }

  // 反转并限幅（左/右轮互换，前进/后退取反）
  int16_t mapped_12 = (speed_34 == 0) ? 0 : -speed_34; // 1/2号电机速度
  int16_t mapped_34 = (speed_12 == 0) ? 0 : -speed_12; // 3/4号电机速度

  // 限幅到[-100, 100]
  if (mapped_12 > 100)  mapped_12 = 100;
  if (mapped_12 < -100) mapped_12 = -100;
  if (mapped_34 > 100)  mapped_34 = 100;
  if (mapped_34 < -100) mapped_34 = -100;

  // 依次下发1/2号电机速度指令
  // dir=0为正转，dir=1为反转，速度为正时dir=0，速度为负时dir=1
  Emm_V5_Vel_Control(1, mapped_12 >= 0 ? 0 : 1, mapped_12 >= 0 ? mapped_12 : -mapped_12, 40, false);
  osDelay(delay_ms);
  Emm_V5_Vel_Control(2, mapped_12 >= 0 ? 0 : 1, mapped_12 >= 0 ? mapped_12 : -mapped_12, 40, false);
  osDelay(delay_ms);

  // 依次下发3/4号电机速度指令
  // 注意3/4号电机方向与1/2号相反：dir=1为正转，dir=0为反转
  Emm_V5_Vel_Control(3, mapped_34 >= 0 ? 1 : 0, mapped_34 >= 0 ? mapped_34 : -mapped_34, 40, false);
  osDelay(delay_ms);
  Emm_V5_Vel_Control(4, mapped_34 >= 0 ? 1 : 0, mapped_34 >= 0 ? mapped_34 : -mapped_34, 40, false);
  osDelay(delay_ms);
}
