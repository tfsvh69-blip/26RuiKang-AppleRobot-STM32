/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    Usart_to_Pi.h
	* @brief   树莓派 USART2 通信基础封装。
	******************************************************************************
	*/
/* USER CODE END Header */
#ifndef __USART_TO_PI_H__
#define __USART_TO_PI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define PI_UART_RX_BUF_LEN  (256u)
#define PI_UART_TX_BUF_LEN  (256u)

/**
  * @brief  启动树莓派串口接收。
  * @note   当前已确认使用 huart2。
  */
void PiUart2_Init(void);
void PiUart2_StartRx(void);

/**
  * @brief  获取一帧串口数据。
  * @note   底层按 UART idle 收帧；协议层负责识别换行和解析文本。
  */
bool PiUart2_GetFrame(uint8_t *out, uint16_t out_len, uint16_t *frame_len);
bool PiUart2_Send(const uint8_t *data, uint16_t len);
bool PiUart2_SendByte(uint8_t data);
void PiUart2_TestSend(void);

void PiUart2_OnRxEvent(uint16_t size);
void PiUart2_OnTxCplt(void);
void PiUart2_OnError(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_TO_PI_H__ */
