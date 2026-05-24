/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdint.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

#define EMM_UART_RX_BUF_LEN  (64u)
#define IMU_UART_RX_BUF_LEN  (130u)

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

typedef struct {
  uint8_t data_id;
  uint8_t data_len;
} payload_data_t;

typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;

  float angle_x;
  float angle_y;
  float angle_z;

  float mag_x;
  float mag_y;
  float mag_z;

  float raw_mag_x;
  float raw_mag_y;
  float raw_mag_z;

  float pitch;
  float roll;
  float yaw;

  float quaternion_data0;
  float quaternion_data1;
  float quaternion_data2;
  float quaternion_data3;

  double latitude;
  double longtidue;
  float altidue;

  float vel_n;
  float vel_e;
  float vel_d;

  uint32_t sample_timestamp;
  uint32_t data_ready_timestamp;
} protocol_info_t;

void Lidar_UartRxStart(void);

extern volatile bool rxFrameFlag;
extern volatile uint8_t rxCmd[EMM_UART_RX_BUF_LEN];
extern volatile uint8_t rxCount;
extern volatile bool imuFrameFlag;
extern protocol_info_t g_output_info;

void Emm_UartRxStart(void);
void Imu_Uart3RxStart(void);
uint8_t Imu_GetFrameReadyAndClear(void);
void usart_SendCmd(const uint8_t *cmd, uint8_t len);
void usart_SendByte(uint16_t data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

