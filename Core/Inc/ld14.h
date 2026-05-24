#ifndef __LD14_H__
#define __LD14_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* LD14(激光测距) 串口协议解析模块
 * - 输入：按字节喂入（来自 SC16IS752 通道A）
 * - 输出：解析成功后更新一组 volatile 变量（Keil Watch 友好）
 */

/* 解析结果（平均值） */
extern volatile uint16_t g_ld14_distance_mm;
extern volatile uint16_t g_ld14_noise;
extern volatile uint32_t g_ld14_peak;
extern volatile uint8_t  g_ld14_confidence;
extern volatile uint32_t g_ld14_intg;
extern volatile int16_t  g_ld14_reftof;

/* 调试计数器：用于定位“有没有在收/有没有通过CRC” */
extern volatile uint32_t g_ld14_rx_bytes_cnt;
extern volatile uint32_t g_ld14_frame_ok_cnt;
extern volatile uint32_t g_ld14_frame_crc_err_cnt;
extern volatile uint8_t  g_ld14_last_crc_calc;
extern volatile uint8_t  g_ld14_last_crc_recv;

void LD14_Reset(void);
void LD14_FeedByte(uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* __LD14_H__ */
