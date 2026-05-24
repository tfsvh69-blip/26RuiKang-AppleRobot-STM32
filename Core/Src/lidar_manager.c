/*
 * lidar_manager.c
 *
 * 底层回调对接伪代码示例：
 * UART4 IDLE 中断回调：  Lidar_ParseFrame(0, Laser1_RxBuf, rx_len);
 * UART5 IDLE 中断回调：  Lidar_ParseFrame(1, Laser2_RxBuf, rx_len);
 * USART6 IDLE 中断回调： Lidar_ParseFrame(2, Laser3_RxBuf, rx_len);
 * SC16IS752 DMA回调：    Lidar_ParseFrame(3, SC16_CHA_RxBuf, rx_len);
 *
 * 无锁化说明：
 * 每路传感器只写自己对应的 g_LidarArray[index]，不同中断源
 * 操作不同索引，天然互不冲突。
 */

#include "lidar_manager.h"
#include <string.h>

#define LIDAR_FRAME_LEN 195u
#define LIDAR_HEADER    0xAAu
#define LIDAR_CMD_DIST  0x02u
//----------------- 数据结构定义 -----------------
LidarSensor_t g_LidarArray[4];

static uint16_t lidar_read_u16_le(const uint8_t *p)
{
    /* 小端16位：p[0]为低字节，p[1]为高字节 */
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t lidar_read_u32_le(const uint8_t *p)
{
    /* 小端32位：p[0]为低字节，p[3]为高字节 */
    return (uint32_t)((uint32_t)p[0] |
                      ((uint32_t)p[1] << 8) |
                      ((uint32_t)p[2] << 16) |
                      ((uint32_t)p[3] << 24));
}

void Lidar_ParseFrame(uint8_t sensor_id, uint8_t *raw_buf, uint16_t buf_len)
{
    uint32_t crc_sum = 0;
    uint32_t crc_sum_no_header = 0;
    uint16_t i;

    if (sensor_id >= 4 || raw_buf == NULL) {
        return;
    }

    if (buf_len != LIDAR_FRAME_LEN) {
        g_LidarArray[sensor_id].status = 2;
        return;
    }

    /* 帧头校验：0~3字节必须为0xAA */
    if (raw_buf[0] != LIDAR_HEADER || raw_buf[1] != LIDAR_HEADER ||
        raw_buf[2] != LIDAR_HEADER || raw_buf[3] != LIDAR_HEADER) {
        g_LidarArray[sensor_id].status = 2;
        return;
    }

    /* 命令字校验（第5字节） */
    if (raw_buf[5] != LIDAR_CMD_DIST) {
        g_LidarArray[sensor_id].status = 2;
        return;
    }

    /* CRC：兼容两种计算方式
     * 1) 字节0~193累加（包含4字节帧头）
     * 2) 字节4~193累加（不含帧头，参考旧实现）
     */
    for (i = 0; i < 194u; ++i) {
        crc_sum += raw_buf[i];
        if (i >= 4u) {
            crc_sum_no_header += raw_buf[i];
        }
    }

    if ((uint8_t)crc_sum != raw_buf[194] && (uint8_t)crc_sum_no_header != raw_buf[194]) {
        g_LidarArray[sensor_id].status = 1;
        return;
    }

    /* 从偏移10开始解析12个点，每点15字节 */
    {
        const uint8_t *p = &raw_buf[10];
        for (i = 0; i < 12u; ++i) {
            /*
             * 指针偏移由协议固定：
             * 0-1 距离, 2-3 噪声, 4-7 强度,
             * 8 置信度, 9-12 积分, 13-14 温度表征值。
             */
            g_LidarArray[sensor_id].points[i].distance   = (int16_t)lidar_read_u16_le(p + 0);
            g_LidarArray[sensor_id].points[i].noise      = lidar_read_u16_le(p + 2);
            g_LidarArray[sensor_id].points[i].peak       = lidar_read_u32_le(p + 4);
            g_LidarArray[sensor_id].points[i].confidence = *(p + 8);
            g_LidarArray[sensor_id].points[i].intg       = lidar_read_u32_le(p + 9);
            g_LidarArray[sensor_id].points[i].reftof     = (int16_t)lidar_read_u16_le(p + 13);
            p += 15;
        }
    }

    /* 时间戳：190~193字节 */
    g_LidarArray[sensor_id].timestamp = lidar_read_u32_le(&raw_buf[190]);
    g_LidarArray[sensor_id].is_updated = 1;
    g_LidarArray[sensor_id].status = 0;
}

uint8_t Lidar_GetData(uint8_t sensor_id, LidarSensor_t *out_data)
{
    if (sensor_id >= 4 || out_data == NULL) {
        return 0;
    }

    if (g_LidarArray[sensor_id].is_updated != 0u) {
        /* 拷贝整帧给上层，并清除更新标志 */
        memcpy(out_data, &g_LidarArray[sensor_id], sizeof(LidarSensor_t));
        g_LidarArray[sensor_id].is_updated = 0;
        return 1;
    }

    return 0;
}

/* ---------------- 字节流解析器 ---------------- */

typedef struct {
    uint8_t buf[195];
    uint16_t idx;
    uint8_t header_cnt;
} LidarStreamParser_t;

static LidarStreamParser_t parser_states[4] = {0};

void Lidar_ProcessStream(uint8_t sensor_id, const uint8_t *stream, uint16_t len)
{
    if (sensor_id >= 4 || !stream || len == 0) return;

    LidarStreamParser_t *state = &parser_states[sensor_id];

    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = stream[i];

        if (state->idx == 0u) {
            if (b == 0xAAu) {
                state->header_cnt++;
                if (state->header_cnt >= 4u) {
                    state->buf[0] = 0xAAu;
                    state->buf[1] = 0xAAu;
                    state->buf[2] = 0xAAu;
                    state->buf[3] = 0xAAu;
                    state->idx = 4u;
                    state->header_cnt = 0u;
                }
            } else {
                state->header_cnt = 0u;
            }
            continue;
        }

        state->buf[state->idx++] = b;
        
        if (state->idx >= 195u) {
            Lidar_ParseFrame(sensor_id, state->buf, 195u);
            state->idx = 0u;
            state->header_cnt = 0u;
        }
    }
}
