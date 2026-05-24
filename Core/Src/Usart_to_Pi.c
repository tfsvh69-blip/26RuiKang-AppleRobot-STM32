
#include "Usart_to_Pi.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "usart.h"

// 发送一段测试数据到树莓派
void PiUart2_TestSend(void)
{
	const char test_str[] = "Hello Pi, this is STM32!\r\n";
	(void)PiUart2_Send((const uint8_t*)test_str, (uint16_t)strlen(test_str));
}

static uint8_t s_pi_rx_buf[PI_UART_RX_BUF_LEN];
static uint8_t s_pi_frame_buf[PI_UART_RX_BUF_LEN];
static uint16_t s_pi_frame_len = 0u;
static volatile bool s_pi_frame_ready = false;

static uint8_t s_pi_tx_buf[PI_UART_TX_BUF_LEN];
static volatile bool s_pi_tx_busy = false;

void PiUart2_Init(void)
{
	PiUart2_StartRx();
}

void PiUart2_StartRx(void)
{
	s_pi_frame_len = 0u;
	s_pi_frame_ready = false;
	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_pi_rx_buf, PI_UART_RX_BUF_LEN);
	if (huart2.hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}

bool PiUart2_GetFrame(uint8_t *out, uint16_t out_len, uint16_t *frame_len)
{
	uint16_t copy_len;

	if ((out == NULL) || (frame_len == NULL)) {
		return false;
	}

	if (s_pi_frame_ready == false) {
		*frame_len = 0u;
		return false;
	}

	copy_len = s_pi_frame_len;
	if (copy_len > out_len) {
		copy_len = out_len;
	}

	if (copy_len > 0u) {
		uint16_t i;
		for (i = 0u; i < copy_len; i++) {
			out[i] = s_pi_frame_buf[i];
		}
	}

	*frame_len = copy_len;
	s_pi_frame_ready = false;
	s_pi_frame_len = 0u;
	return true;
}

bool PiUart2_Send(const uint8_t *data, uint16_t len)
{
	uint16_t i;

	if ((data == NULL) || (len == 0u)) {
		return false;
	}

	if (len > PI_UART_TX_BUF_LEN) {
		len = (uint16_t)PI_UART_TX_BUF_LEN;
	}

	if (s_pi_tx_busy == true) {
		return false;
	}

	for (i = 0u; i < len; i++) {
		s_pi_tx_buf[i] = data[i];
	}

	s_pi_tx_busy = true;
	if (HAL_UART_Transmit_DMA(&huart2, s_pi_tx_buf, len) != HAL_OK) {
		s_pi_tx_busy = false;
		return false;
	}

	return true;
}

bool PiUart2_SendByte(uint8_t data)
{
	return PiUart2_Send(&data, 1u);
}

void PiUart2_OnRxEvent(uint16_t size)
{
	uint16_t copy_len = size;
	uint16_t i;

	if (copy_len == 0u) {
		return;
	}

	if (copy_len > PI_UART_RX_BUF_LEN) {
		copy_len = PI_UART_RX_BUF_LEN;
	}

	for (i = 0u; i < copy_len; i++) {
		s_pi_frame_buf[i] = s_pi_rx_buf[i];
	}

	s_pi_frame_len = copy_len;
	s_pi_frame_ready = true;

	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_pi_rx_buf, PI_UART_RX_BUF_LEN);
	if (huart2.hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}

void PiUart2_OnTxCplt(void)
{
	s_pi_tx_busy = false;
}

void PiUart2_OnError(void)
{
	s_pi_tx_busy = false;
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	__HAL_UART_CLEAR_FEFLAG(&huart2);
	__HAL_UART_CLEAR_NEFLAG(&huart2);
	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_pi_rx_buf, PI_UART_RX_BUF_LEN);
	if (huart2.hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}
