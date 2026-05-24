/**
 * @file sbus.h
 * @brief SBUS protocol parser and channel data management.
 */

#ifndef __SBUS_H__
#define __SBUS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_CHANNELS 10

/* Keil Watch 友好：允许在任意编译单元直接观察通道数组。 */
extern volatile uint16_t sbus_channels[SBUS_CHANNELS];

/**
 * @brief Initialize the SBUS parser
 */
void SBUS_Init(void);

/**
 * @brief Process incoming SBUS byte stream
 * @param data Array of bytes received
 * @param len  Length of the data array
 */
void SBUS_ProcessStream(const uint8_t *data, uint16_t len);

/**
 * @brief Get a specific SBUS channel value
 * @param channel_idx Index of the channel (0 to 9)
 * @return 11-bit channel value (typically 172-1811, center ~1024)
 */
uint16_t SBUS_GetChannel(uint8_t channel_idx);

/**
 * @brief Check if the last frame had the frame lost indicator flag set
 * @return 1 if frame lost, 0 otherwise
 */
uint8_t SBUS_IsFrameLost(void);

/**
 * @brief Get statistics for valid and error frames
 * @param ok_cnt  Pointer to store the valid frame count (can be NULL)
 * @param err_cnt Pointer to store the error frame count (can be NULL)
 */
void SBUS_GetStats(uint32_t *ok_cnt, uint32_t *err_cnt);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H__ */
