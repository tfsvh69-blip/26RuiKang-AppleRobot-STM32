
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
static uint8_t s_pi_rx_ring[PI_UART_RX_BUF_LEN];
static volatile uint16_t s_pi_rx_head = 0u;
static volatile uint16_t s_pi_rx_tail = 0u;
static volatile bool s_pi_rx_overflow = false;

static uint8_t s_pi_tx_buf[PI_UART_TX_BUF_LEN];
static volatile bool s_pi_tx_busy = false;

void PiUart2_Init(void)
{
	PiUart2_StartRx();
}

void PiUart2_StartRx(void)
{
	s_pi_rx_head = 0u;
	s_pi_rx_tail = 0u;
	s_pi_rx_overflow = false;
	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_pi_rx_buf, PI_UART_RX_BUF_LEN);
	if (huart2.hdmarx != NULL) {
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}

bool PiUart2_GetFrame(uint8_t *out, uint16_t out_len, uint16_t *frame_len)
{
	uint16_t copy_len = 0u;
	uint32_t primask;

	if ((out == NULL) || (frame_len == NULL)) {
		return false;
	}

	if (out_len == 0u) {
		*frame_len = 0u;
		return false;
	}

	primask = __get_PRIMASK();
	__disable_irq();

	while ((s_pi_rx_tail != s_pi_rx_head) && (copy_len < out_len)) {
		out[copy_len] = s_pi_rx_ring[s_pi_rx_tail];
		s_pi_rx_tail = (uint16_t)((s_pi_rx_tail + 1u) % PI_UART_RX_BUF_LEN);
		copy_len++;
	}

	if (primask == 0u) {
		__enable_irq();
	}

	*frame_len = copy_len;
	return (copy_len > 0u);
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
		uint16_t next_head = (uint16_t)((s_pi_rx_head + 1u) % PI_UART_RX_BUF_LEN);

		if (next_head == s_pi_rx_tail) {
			s_pi_rx_tail = (uint16_t)((s_pi_rx_tail + 1u) % PI_UART_RX_BUF_LEN);
			s_pi_rx_overflow = true;
		}

		s_pi_rx_ring[s_pi_rx_head] = s_pi_rx_buf[i];
		s_pi_rx_head = next_head;
	}

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
