/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "lidar_manager.h"
#include "Usart_to_Pi.h"

#define LIDAR_UART_RX_LEN  (195u)

#define IMU_PROTOCOL_FIRST_BYTE          (0x59u)
#define IMU_PROTOCOL_SECOND_BYTE         (0x53u)
#define IMU_PROTOCOL_MIN_LEN             (7u)
#define IMU_PAYLOAD_POS                  (5u)
#define IMU_SINGLE_DATA_BYTES            (4u)

#define IMU_ACCEL_ID                     (0x10u)
#define IMU_ANGLE_ID                     (0x20u)
#define IMU_MAGNETIC_ID                  (0x30u)
#define IMU_RAW_MAGNETIC_ID              (0x31u)
#define IMU_EULER_ID                     (0x40u)
#define IMU_QUATERNION_ID                (0x41u)
#define IMU_SAMPLE_TIMESTAMP_ID          (0x51u)
#define IMU_DATA_READY_TIMESTAMP_ID      (0x52u)
#define IMU_LOCATION_ID                  (0x60u)
#define IMU_SPEED_ID                     (0x70u)

#define IMU_ACCEL_DATA_LEN               (12u)
#define IMU_ANGLE_DATA_LEN               (12u)
#define IMU_MAGNETIC_DATA_LEN            (12u)
#define IMU_MAGNETIC_RAW_DATA_LEN        (12u)
#define IMU_EULER_DATA_LEN               (12u)
#define IMU_QUATERNION_DATA_LEN          (16u)
#define IMU_SAMPLE_TIMESTAMP_DATA_LEN    (4u)
#define IMU_DATA_READY_TIMESTAMP_DATA_LEN (4u)
#define IMU_LOCATION_DATA_LEN            (12u)
#define IMU_SPEED_DATA_LEN               (12u)

#define IMU_NOT_MAG_DATA_FACTOR          (0.000001f)
#define IMU_MAG_RAW_DATA_FACTOR          (0.001f)
#define IMU_LONG_LAT_DATA_FACTOR         (0.0000001)
#define IMU_ALT_DATA_FACTOR              (0.001f)
#define IMU_SPEED_DATA_FACTOR            (0.001f)
#define USART1_TX_DMA_BUF_LEN            (128u)

static uint8_t s_uart4_rx_buf[LIDAR_UART_RX_LEN];
static uint8_t s_uart5_rx_buf[LIDAR_UART_RX_LEN];
static uint8_t s_usart6_rx_buf[LIDAR_UART_RX_LEN];
static uint8_t s_usart3_rx_buf[IMU_UART_RX_BUF_LEN];
static uint8_t s_usart3_frame_buf[IMU_UART_RX_BUF_LEN];
static uint16_t s_usart3_rx_count = 0u;
static uint16_t s_usart3_expect_len = 0u;
static uint8_t s_usart3_last_byte = 0u;

volatile bool rxFrameFlag = false;
volatile uint8_t rxCmd[EMM_UART_RX_BUF_LEN] = {0};
volatile uint8_t rxCount = 0;
volatile bool imuFrameFlag = false;

protocol_info_t g_output_info = {0};

static uint8_t s_usart1_rx_buf[EMM_UART_RX_BUF_LEN];
static uint8_t s_usart1_tx_buf[USART1_TX_DMA_BUF_LEN];
static volatile bool s_usart1_tx_busy = false;

static int32_t Imu_ReadI32LE(const uint8_t *data)
{
  return ((int32_t)data[0]) | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 24);
}

static uint32_t Imu_ReadU32LE(const uint8_t *data)
{
  return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint16_t Imu_CalcChecksum(const uint8_t *data, uint16_t len)
{
  uint8_t check_a = 0u;
  uint8_t check_b = 0u;
  uint16_t i;

  for (i = 0u; i < len; i++) {
    check_a = (uint8_t)(check_a + data[i]);
    check_b = (uint8_t)(check_b + check_a);
  }

  return (uint16_t)(((uint16_t)check_b << 8) | (uint16_t)check_a);
}

static uint8_t Imu_CheckDataLenById(uint8_t id, uint8_t len, const uint8_t *data)
{
  switch (id) {
    case IMU_ACCEL_ID:
      if (len == IMU_ACCEL_DATA_LEN) {
        g_output_info.accel_x = (float)Imu_ReadI32LE(data) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.accel_y = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.accel_z = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_NOT_MAG_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_ANGLE_ID:
      if (len == IMU_ANGLE_DATA_LEN) {
        g_output_info.angle_x = (float)Imu_ReadI32LE(data) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.angle_y = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.angle_z = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_NOT_MAG_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_MAGNETIC_ID:
      if (len == IMU_MAGNETIC_DATA_LEN) {
        g_output_info.mag_x = (float)Imu_ReadI32LE(data) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.mag_y = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.mag_z = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_NOT_MAG_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_RAW_MAGNETIC_ID:
      if (len == IMU_MAGNETIC_RAW_DATA_LEN) {
        g_output_info.raw_mag_x = (float)Imu_ReadI32LE(data) * IMU_MAG_RAW_DATA_FACTOR;
        g_output_info.raw_mag_y = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_MAG_RAW_DATA_FACTOR;
        g_output_info.raw_mag_z = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_MAG_RAW_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_EULER_ID:
      if (len == IMU_EULER_DATA_LEN) {
        g_output_info.pitch = (float)Imu_ReadI32LE(data) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.roll = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.yaw = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_NOT_MAG_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_QUATERNION_ID:
      if (len == IMU_QUATERNION_DATA_LEN) {
        g_output_info.quaternion_data0 = (float)Imu_ReadI32LE(data) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.quaternion_data1 = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.quaternion_data2 = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_NOT_MAG_DATA_FACTOR;
        g_output_info.quaternion_data3 = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 3u) * IMU_NOT_MAG_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_LOCATION_ID:
      if (len == IMU_LOCATION_DATA_LEN) {
        g_output_info.latitude = (double)Imu_ReadI32LE(data) * IMU_LONG_LAT_DATA_FACTOR;
        g_output_info.longtidue = (double)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_LONG_LAT_DATA_FACTOR;
        g_output_info.altidue = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_ALT_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_SPEED_ID:
      if (len == IMU_SPEED_DATA_LEN) {
        g_output_info.vel_n = (float)Imu_ReadI32LE(data) * IMU_SPEED_DATA_FACTOR;
        g_output_info.vel_e = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES) * IMU_SPEED_DATA_FACTOR;
        g_output_info.vel_d = (float)Imu_ReadI32LE(data + IMU_SINGLE_DATA_BYTES * 2u) * IMU_SPEED_DATA_FACTOR;
        return 1u;
      }
      break;

    case IMU_SAMPLE_TIMESTAMP_ID:
      if (len == IMU_SAMPLE_TIMESTAMP_DATA_LEN) {
        g_output_info.sample_timestamp = Imu_ReadU32LE(data);
        return 1u;
      }
      break;

    case IMU_DATA_READY_TIMESTAMP_ID:
      if (len == IMU_DATA_READY_TIMESTAMP_DATA_LEN) {
        g_output_info.data_ready_timestamp = Imu_ReadU32LE(data);
        return 1u;
      }
      break;

    default:
      break;
  }

  return 0u;
}

static void Imu_ParseFrame(const uint8_t *frame, uint16_t frame_len)
{
  uint16_t payload_len;
  uint16_t calc_sum;
  uint16_t recv_sum;
  uint16_t pos;

  if ((frame == NULL) || (frame_len < IMU_PROTOCOL_MIN_LEN)) {
    return;
  }

  if ((frame[0] != IMU_PROTOCOL_FIRST_BYTE) || (frame[1] != IMU_PROTOCOL_SECOND_BYTE)) {
    return;
  }

  payload_len = frame[4];
  if ((uint16_t)(payload_len + IMU_PROTOCOL_MIN_LEN) != frame_len) {
    return;
  }

  calc_sum = Imu_CalcChecksum(&frame[2], (uint16_t)(payload_len + 3u));
  recv_sum = (uint16_t)frame[5u + payload_len] | ((uint16_t)frame[6u + payload_len] << 8);
  if (calc_sum != recv_sum) {
    return;
  }

  pos = IMU_PAYLOAD_POS;
  while ((payload_len > 0u) && (pos < (frame_len - 2u))) {
    uint8_t data_id;
    uint8_t data_len;
    const uint8_t *payload_data;

    if ((uint16_t)(pos + 2u) > (frame_len - 2u)) {
      break;
    }

    data_id = frame[pos];
    data_len = frame[pos + 1u];
    payload_data = &frame[pos + 2u];

    if ((uint16_t)(pos + 2u + data_len) > (frame_len - 2u)) {
      break;
    }

    if (Imu_CheckDataLenById(data_id, data_len, payload_data) == 1u) {
      pos = (uint16_t)(pos + 2u + data_len);
      payload_len = (uint16_t)(payload_len - 2u - data_len);
    } else {
      pos = (uint16_t)(pos + 1u);
      payload_len = (uint16_t)(payload_len - 1u);
    }
  }

  imuFrameFlag = true;
}

static void Imu_ResetRxState(void)
{
  s_usart3_rx_count = 0u;
  s_usart3_expect_len = 0u;
}

static void Imu_RxByte(uint8_t rx)
{
  if (((s_usart3_last_byte == IMU_PROTOCOL_FIRST_BYTE) && (rx == IMU_PROTOCOL_SECOND_BYTE)) || (s_usart3_rx_count > 0u)) {
    if (s_usart3_rx_count == 0u) {
      s_usart3_frame_buf[0] = IMU_PROTOCOL_FIRST_BYTE;
      s_usart3_frame_buf[1] = IMU_PROTOCOL_SECOND_BYTE;
      s_usart3_rx_count = 2u;
      s_usart3_expect_len = 0u;
    } else {
      if (s_usart3_rx_count >= IMU_UART_RX_BUF_LEN) {
        Imu_ResetRxState();
      } else {
        s_usart3_frame_buf[s_usart3_rx_count] = rx;
        s_usart3_rx_count++;
      }
    }

    if (s_usart3_rx_count == IMU_PAYLOAD_POS) {
      uint16_t total_len = (uint16_t)(s_usart3_frame_buf[4] + IMU_PROTOCOL_MIN_LEN);
      if ((total_len < IMU_PROTOCOL_MIN_LEN) || (total_len > IMU_UART_RX_BUF_LEN)) {
        Imu_ResetRxState();
      } else {
        s_usart3_expect_len = total_len;
      }
    }

    if ((s_usart3_expect_len > 0u) && (s_usart3_rx_count == s_usart3_expect_len)) {
      Imu_ParseFrame(s_usart3_frame_buf, s_usart3_expect_len);
      Imu_ResetRxState();
    }
  }

  s_usart3_last_byte = rx;
}

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  Emm_UartRxStart();

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  PiUart2_Init();

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  Imu_Uart3RxStart();

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 230400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void Emm_UartRxStart(void)
{
  rxFrameFlag = false;
  rxCount = 0u;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_usart1_rx_buf, EMM_UART_RX_BUF_LEN);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void Imu_Uart3RxStart(void)
{
  imuFrameFlag = false;
  Imu_ResetRxState();
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_usart3_rx_buf, IMU_UART_RX_BUF_LEN);
  if (huart3.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

uint8_t Imu_GetFrameReadyAndClear(void)
{
  uint8_t ready = (imuFrameFlag == true) ? 1u : 0u;
  imuFrameFlag = false;
  return ready;
}

void usart_SendCmd(const uint8_t *cmd, uint8_t len)
{
  uint8_t i;

  if ((cmd == NULL) || (len == 0u)) {
    return;
  }

  if (len > USART1_TX_DMA_BUF_LEN) {
    len = (uint8_t)USART1_TX_DMA_BUF_LEN;
  }

  if (s_usart1_tx_busy == true) {
    return;
  }

  for (i = 0u; i < len; i++) {
    s_usart1_tx_buf[i] = cmd[i];
  }

  s_usart1_tx_busy = true;
  if (HAL_UART_Transmit_DMA(&huart1, s_usart1_tx_buf, len) != HAL_OK) {
    s_usart1_tx_busy = false;
  }
}

void usart_SendByte(uint16_t data)
{
  uint8_t byte = (uint8_t)(data & 0xFFu);
  usart_SendCmd(&byte, 1u);
}

void Lidar_UartRxStart(void)
{
  /* 使用 IDLE+DMA 接收一帧，长度以回调里的 RxLen 为准 */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_uart4_rx_buf, LIDAR_UART_RX_LEN);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_rx_buf, LIDAR_UART_RX_LEN);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_usart6_rx_buf, LIDAR_UART_RX_LEN);

  /* 关闭 DMA 半传中断，减少无用打断 */
  if (huart4.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
  }
  if (huart5.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
  }
  if (huart6.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  /* IDLE触发：收到一段数据，size 为 DMA 实际接收长度 */
  if (size == 0u) {
    return;
  }

  if (huart->Instance == UART4) {
    Lidar_ParseFrame(0u, s_uart4_rx_buf, size);
    /* 关键：重新启动DMA接收，避免停滞 */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_uart4_rx_buf, LIDAR_UART_RX_LEN);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == UART5) {
    Lidar_ParseFrame(1u, s_uart5_rx_buf, size);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_rx_buf, LIDAR_UART_RX_LEN);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART6) {
    Lidar_ParseFrame(2u, s_usart6_rx_buf, size);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_usart6_rx_buf, LIDAR_UART_RX_LEN);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART1) {
    uint16_t copy_len = size;
    uint16_t i;

    if (copy_len > EMM_UART_RX_BUF_LEN) {
      copy_len = EMM_UART_RX_BUF_LEN;
    }

    for (i = 0u; i < copy_len; i++) {
      rxCmd[i] = s_usart1_rx_buf[i];
    }

    rxCount = (uint8_t)copy_len;
    rxFrameFlag = true;

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_usart1_rx_buf, EMM_UART_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART2) {
    PiUart2_OnRxEvent(size);
  } else if (huart->Instance == USART3) {
    uint16_t i;

    for (i = 0u; i < size; i++) {
      Imu_RxByte(s_usart3_rx_buf[i]);
    }

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_usart3_rx_buf, IMU_UART_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    s_usart1_tx_busy = false;
  } else if (huart->Instance == USART2) {
    PiUart2_OnTxCplt();
  }
}

/* UART错误回调：处理溢出、帧错误等异常 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* 清除错误标志并重启DMA接收 */
  if (huart->Instance == UART4) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_uart4_rx_buf, LIDAR_UART_RX_LEN);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == UART5) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_rx_buf, LIDAR_UART_RX_LEN);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART6) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_usart6_rx_buf, LIDAR_UART_RX_LEN);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    s_usart1_tx_busy = false;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_usart1_rx_buf, EMM_UART_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  } else if (huart->Instance == USART2) {
    PiUart2_OnError();
  } else if (huart->Instance == USART3) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_usart3_rx_buf, IMU_UART_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

/* USER CODE END 1 */

